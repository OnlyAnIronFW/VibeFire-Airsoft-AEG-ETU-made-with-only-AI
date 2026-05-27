#pragma once

#include "autotrigger/hal/iranging.h"
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace autotrigger {

// ============================================================================
// Ranging — real driver for JRT TOF01 UART laser + monocular fallback fusion
// ============================================================================

class Ranging : public IRanging {
 public:
  /// UART read callback (injectable for testing).
  /// Signature: read(uint8_t* buffer, size_t max_len) → bytes read (-1 on error)
  using UartReadFn = std::function<int(uint8_t*, size_t)>;

  explicit Ranging(const std::string& uart_path = "/dev/ttyS1");
  ~Ranging() override;

  // ---- IRanging interface ------------------------------------------------
  bool init() override;
  float get_distance() override;
  bool is_healthy() override;

  // ---- Monocular fallback ------------------------------------------------
  /// Feed a monocular bbox detection for fallback / fusion.
  /// @param bbox_height_px  bounding-box height in pixels
  /// @param focal_length_px  camera focal length in pixels
  void set_monocular_input(float bbox_height_px, float focal_length_px);

  // ---- Configuration ----------------------------------------------------
  void set_laser_confidence(float conf) { laser_confidence_ = conf; }
  void set_timeout_ms(int ms) { timeout_ms_ = ms; }

  // ---- Test injection ----------------------------------------------------
  /// Replace UART read with a mock callback.
  void set_uart_read_fn(UartReadFn fn) { uart_read_fn_ = std::move(fn); }

  // ---- Status accessors --------------------------------------------------
  bool has_low_confidence() const { return low_confidence_; }
  float get_last_laser_distance() const { return last_laser_distance_; }
  float get_last_monocular_distance() const { return last_monocular_distance_; }
  float get_fused_distance() const { return fused_distance_; }

  // ---- Protocol constants ------------------------------------------------
  static constexpr int    JRT_FRAME_SIZE   = 8;
  static constexpr uint8_t JRT_HEADER      = 0x5A;
  static constexpr int    DEFAULT_TIMEOUT_MS = 200;

 private:
  // UART helpers
  bool open_uart();
  void close_uart();
  int  read_uart(uint8_t* buf, size_t len);

  // Frame parsing
  static uint8_t calc_checksum(const uint8_t* frame);
  static bool    parse_frame(const uint8_t* frame, float& distance_m);

  // Fusion / fallback
  float compute_monocular_distance(float bbox_h, float focal);
  static float fuse(float laser_d, float mono_d, float laser_conf);

  // Bbox-height variance tracking
  void update_height_variance(float bbox_h);

  // ---- Members -----------------------------------------------------------
  std::string uart_path_;
  int         uart_fd_ = -1;
  UartReadFn  uart_read_fn_;

  float last_laser_distance_     = 0.0f;
  float last_monocular_distance_ = 0.0f;
  float fused_distance_          = 0.0f;
  float laser_confidence_        = 0.9f;   // Laser trusted more
  float target_height_m_         = 1.7f;   // Typical human height

  bool healthy_              = false;
  bool low_confidence_       = false;
  bool has_monocular_input_  = false;
  bool has_valid_frame_      = false;

  int timeout_ms_ = DEFAULT_TIMEOUT_MS;
  std::chrono::steady_clock::time_point last_read_time_;

  static constexpr int    MONO_HISTORY_SIZE = 5;
  std::vector<float> mono_height_history_;
};

// ============================================================================
// RangingMock — programmable mock for unit testing
// ============================================================================

class RangingMock : public IRanging {
 public:
  void set_distance(float d) { distance_ = d; }
  void set_healthy(bool h) { healthy_ = h; }
  void set_low_confidence(bool lc) { low_confidence_ = lc; }

  bool   init() override { return true; }
  float  get_distance() override { return distance_; }
  bool   is_healthy() override { return healthy_; }
  bool   has_low_confidence() const { return low_confidence_; }

 private:
  float distance_       = 0.0f;
  bool  healthy_        = true;
  bool  low_confidence_ = false;
};

}  // namespace autotrigger
