#include "autotrigger/ranging.h"

#include <cstdint>
#include <cmath>

// ============================================================================
// Platform-guarded UART headers
// ============================================================================
#ifndef MOCK_MODE
#include <unistd.h>    // read, close
#include <fcntl.h>     // open, O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h>   // termios, cfsetospeed, etc.
#include <errno.h>     // errno, EAGAIN
#endif

namespace autotrigger {

// ============================================================================
// Construction / destruction
// ============================================================================

Ranging::Ranging(const std::string& uart_path)
    : uart_path_(uart_path) {
  mono_height_history_.reserve(MONO_HISTORY_SIZE);
}

Ranging::~Ranging() {
  close_uart();
}

// ============================================================================
// IRanging interface
// ============================================================================

bool Ranging::init() {
  return open_uart();
}

float Ranging::get_distance() {
  // ---- 1. Try to read a UART frame from the laser ------------------------
  uint8_t frame[JRT_FRAME_SIZE];
  float   laser_dist = 0.0f;
  bool    laser_ok   = false;

  int n = read_uart(frame, JRT_FRAME_SIZE);
  if (n == JRT_FRAME_SIZE && frame[0] == JRT_HEADER) {
    laser_ok = parse_frame(frame, laser_dist);
  }

  if (laser_ok) {
    last_laser_distance_ = laser_dist;
    last_read_time_      = std::chrono::steady_clock::now();
    has_valid_frame_     = true;
  }

  // ---- 2. Compute fused distance -----------------------------------------
  if (last_laser_distance_ > 0.0f && has_monocular_input_) {
    fused_distance_ = fuse(last_laser_distance_,
                           last_monocular_distance_,
                           laser_confidence_);
  } else if (has_monocular_input_ && last_monocular_distance_ > 0.0f) {
    fused_distance_ = last_monocular_distance_;
  } else if (has_valid_frame_) {
    fused_distance_ = last_laser_distance_;
  }
  // else fused_distance_ stays at 0 (no data at all)

  return fused_distance_;
}

bool Ranging::is_healthy() {
  if (!has_valid_frame_) return false;
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::steady_clock::now() - last_read_time_)
                     .count();
  return elapsed < timeout_ms_;
}

// ============================================================================
// Monocular input
// ============================================================================

void Ranging::set_monocular_input(float bbox_height_px, float focal_length_px) {
  last_monocular_distance_ = compute_monocular_distance(bbox_height_px,
                                                        focal_length_px);
  has_monocular_input_     = true;

  update_height_variance(bbox_height_px);
}

// ============================================================================
// UART helpers
// ============================================================================

bool Ranging::open_uart() {
#ifdef MOCK_MODE
  // In mock/test mode we never touch real hardware.
  // The test must provide a callback via set_uart_read_fn().
  return true;
#else
  uart_fd_ = ::open(uart_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (uart_fd_ < 0) return false;

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(uart_fd_, &tty) != 0) {
    close_uart();
    return false;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(INLCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
    close_uart();
    return false;
  }

  return true;
#endif
}

void Ranging::close_uart() {
#ifndef MOCK_MODE
  if (uart_fd_ >= 0) {
    ::close(uart_fd_);
    uart_fd_ = -1;
  }
#endif
}

int Ranging::read_uart(uint8_t* buf, size_t len) {
  // If a test callback is installed, use it.
  if (uart_read_fn_) {
    return uart_read_fn_(buf, len);
  }

#ifdef MOCK_MODE
  return -1;  // No real UART available on x86
#else
  if (uart_fd_ < 0) return -1;
  ssize_t n = ::read(uart_fd_, buf, len);
  if (n < 0 && errno == EAGAIN) return 0;   // No data right now
  return static_cast<int>(n);
#endif
}

// ============================================================================
// JRT TOF01 frame parsing
//
// 8-byte frame format:
//   [0] = 0x5A  (sync header)
//   [1] = distance low byte  (cm)
//   [2] = distance high byte (cm)
//   [3] = reserved (0x00)
//   [4] = reserved (0x00)
//   [5] = reserved (0x00)
//   [6] = status   (0x00 = OK)
//   [7] = checksum (sum of bytes 0-6, modulo 256)
// ============================================================================

uint8_t Ranging::calc_checksum(const uint8_t* frame) {
  uint8_t sum = 0;
  for (int i = 0; i < 7; ++i) {
    sum += frame[i];
  }
  return sum;
}

bool Ranging::parse_frame(const uint8_t* frame, float& distance_m) {
  if (frame[0] != JRT_HEADER) return false;
  if (frame[7] != calc_checksum(frame)) return false;

  uint16_t dist_cm = (static_cast<uint16_t>(frame[2]) << 8) | frame[1];
  distance_m       = static_cast<float>(dist_cm) / 100.0f;
  return true;
}

// ============================================================================
// Monocular distance formula
//   distance = focal_length_px × target_height_m / bbox_height_px
// ============================================================================

float Ranging::compute_monocular_distance(float bbox_h, float focal) {
  if (bbox_h < 1.0f) return 0.0f;  // degenerate
  return focal * target_height_m_ / bbox_h;
}

// ============================================================================
// Sensor fusion
//   distance = w × laser + (1 − w) × mono   where w = laser_confidence
// ============================================================================

float Ranging::fuse(float laser_d, float mono_d, float laser_conf) {
  bool laser_valid = (laser_d > 0.0f);
  bool mono_valid  = (mono_d > 0.0f);

  if (laser_valid && mono_valid) {
    return laser_conf * laser_d + (1.0f - laser_conf) * mono_d;
  }
  if (laser_valid) return laser_d;
  if (mono_valid)  return mono_d;
  return 0.0f;
}

// ============================================================================
// Bbox-height variance tracking
//   Sets low_confidence_ when coefficient of variation > 30 %
// ============================================================================

void Ranging::update_height_variance(float bbox_h) {
  mono_height_history_.push_back(bbox_h);
  if (mono_height_history_.size() > MONO_HISTORY_SIZE) {
    mono_height_history_.erase(mono_height_history_.begin());
  }

  // Need at least 3 samples for a meaningful variance estimate.
  if (mono_height_history_.size() < 3) return;

  float mean = 0.0f;
  for (float h : mono_height_history_) mean += h;
  mean /= static_cast<float>(mono_height_history_.size());

  float variance = 0.0f;
  for (float h : mono_height_history_) {
    float diff = h - mean;
    variance += diff * diff;
  }
  variance /= static_cast<float>(mono_height_history_.size());

  float stddev     = std::sqrt(variance);
  float cv         = stddev / mean;
  low_confidence_  = (cv > 0.30f);
}

}  // namespace autotrigger
