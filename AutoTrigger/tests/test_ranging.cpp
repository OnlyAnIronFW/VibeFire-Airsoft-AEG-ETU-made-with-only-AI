#include <gtest/gtest.h>
#include "autotrigger/ranging.h"
#include <thread>
#include <chrono>
#include <cstring>

using namespace autotrigger;

// ---------------------------------------------------------------------------
// Test 1: Valid UART frame → correct distance parsed
//   JRT TOF01 frame encoding 42.3 m → get_distance() == 42.3
//   Frame: 5A 86 10 00 00 00 00 F0
//   42.3 m = 4230 cm → dist_L=0x86, dist_H=0x10, checksum=0xF0
// ---------------------------------------------------------------------------
TEST(RangingTest, ValidUartFrameParsesDistance) {
  Ranging ranging("/dev/null");

  uint8_t valid_frame[8] = {0x5A, 0x86, 0x10, 0x00, 0x00, 0x00, 0x00, 0xF0};
  ranging.set_uart_read_fn(
      [valid_frame](uint8_t* buf, size_t len) -> int {
        if (len >= 8) {
          std::memcpy(buf, valid_frame, 8);
          return 8;
        }
        return -1;
      });

  ranging.init();
  float d = ranging.get_distance();
  EXPECT_NEAR(d, 42.3f, 0.1f);
}

// ---------------------------------------------------------------------------
// Test 2: CRC error → frame rejected, previous value maintained
//   First call returns valid 42.3m frame, second returns CRC-bad frame
// ---------------------------------------------------------------------------
TEST(RangingTest, CrcErrorRetainsPreviousValue) {
  Ranging ranging("/dev/null");

  int call = 0;
  ranging.set_uart_read_fn(
      [&call](uint8_t* buf, size_t len) -> int {
        if (len < 8) return -1;
        if (call++ == 0) {
          // Valid frame: 42.3m
          uint8_t valid[8] = {0x5A, 0x86, 0x10, 0x00, 0x00, 0x00, 0x00, 0xF0};
          std::memcpy(buf, valid, 8);
        } else {
          // Bad CRC: header OK but checksum is deliberately wrong
          uint8_t bad[8] = {0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
          std::memcpy(buf, bad, 8);
        }
        return 8;
      });

  ranging.init();
  float d1 = ranging.get_distance();
  EXPECT_NEAR(d1, 42.3f, 0.1f);

  float d2 = ranging.get_distance();
  EXPECT_NEAR(d2, 42.3f, 0.1f);  // Previous value retained
}

// ---------------------------------------------------------------------------
// Test 3: Timeout (200ms no response) → is_healthy() returns false
//   Get one valid frame, stop responding, wait past short timeout
// ---------------------------------------------------------------------------
TEST(RangingTest, TimeoutSetsHealthyFalse) {
  Ranging ranging("/dev/null");
  ranging.set_timeout_ms(1);  // Very short for test

  ranging.set_uart_read_fn(
      [](uint8_t* buf, size_t len) -> int {
        if (len < 8) return -1;
        uint8_t valid[8] = {0x5A, 0x86, 0x10, 0x00, 0x00, 0x00, 0x00, 0xF0};
        std::memcpy(buf, valid, 8);
        return 8;
      });

  ranging.init();
  float d = ranging.get_distance();
  EXPECT_NEAR(d, 42.3f, 0.1f);
  EXPECT_TRUE(ranging.is_healthy());

  // Stop providing data — simulate sensor dropout
  ranging.set_uart_read_fn(
      [](uint8_t*, size_t) -> int { return 0; });

  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  float d2 = ranging.get_distance();  // read fails, no update
  EXPECT_NEAR(d2, 42.3f, 0.1f);       // Last valid value returned
  EXPECT_FALSE(ranging.is_healthy()); // Timed out
}

// ---------------------------------------------------------------------------
// Test 4: Monocular fallback
//   bbox_h=200 px, focal=530 px → d = 530×1.7/200 ≈ 4.505 m (±15 %)
// ---------------------------------------------------------------------------
TEST(RangingTest, MonocularFallbackCalculation) {
  Ranging ranging("/dev/null");
  ranging.set_uart_read_fn(
      [](uint8_t*, size_t) -> int { return -1; });
  ranging.init();

  ranging.set_monocular_input(200.0f, 530.0f);

  float expected = 530.0f * 1.7f / 200.0f;  // 4.505
  float d = ranging.get_distance();
  EXPECT_NEAR(d, expected, expected * 0.15f);  // ±15 % tolerance
}

// ---------------------------------------------------------------------------
// Test 5: Fused mode — weighted average (trust laser more)
//   Laser = 40 m (0.9 weight), monocular ≈ 50 m (0.1 weight)
//   Fused = 0.9×40 + 0.1×50 = 41 m
// ---------------------------------------------------------------------------
TEST(RangingTest, FusedWeightedAverage) {
  Ranging ranging("/dev/null");
  ranging.set_laser_confidence(0.9f);

  // Laser callback returns 40 m = 4000 cm = 0x0FA0
  ranging.set_uart_read_fn(
      [](uint8_t* buf, size_t len) -> int {
        if (len < 8) return -1;
        uint8_t frame[8] = {0x5A, 0xA0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00};
        frame[7] = 0;
        for (int i = 0; i < 7; ++i) frame[7] += frame[i];
        std::memcpy(buf, frame, 8);
        return 8;
      });
  ranging.init();

  // Monocular height that yields ≈ 50 m
  float mono_bh = 530.0f * 1.7f / 50.0f;  // 18.02 px
  ranging.set_monocular_input(mono_bh, 530.0f);

  float mono_actual   = 530.0f * 1.7f / mono_bh;
  float expected      = 0.9f * 40.0f + 0.1f * mono_actual;
  float d             = ranging.get_distance();

  EXPECT_NEAR(d, expected, 0.5f);
}

// ---------------------------------------------------------------------------
// Test 6: Bbox height variance >30 % → low confidence flag
//   3 stable heights → no flag; then add variable heights → flag set
// ---------------------------------------------------------------------------
TEST(RangingTest, HeightVarianceTriggersLowConfidence) {
  Ranging ranging("/dev/null");
  ranging.set_uart_read_fn(
      [](uint8_t*, size_t) -> int { return -1; });

  // 3 stable heights → CV ≈ 2 % → no low-confidence flag
  ranging.set_monocular_input(200.0f, 530.0f);
  ranging.set_monocular_input(205.0f, 530.0f);
  ranging.set_monocular_input(195.0f, 530.0f);
  EXPECT_FALSE(ranging.has_low_confidence());

  // Add two more with large swings → CV ≈ 32 % (>30 %) → flag
  ranging.set_monocular_input(100.0f, 530.0f);
  ranging.set_monocular_input(300.0f, 530.0f);
  EXPECT_TRUE(ranging.has_low_confidence());
}
