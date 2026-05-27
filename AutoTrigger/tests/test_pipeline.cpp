#include "autotrigger/pipeline.h"
#include "autotrigger/display.h"

#include <gtest/gtest.h>

#include <cstdint>

// ============================================================================
// Test doubles
// ============================================================================

/// Trigger subclass that lets tests inject is_held() and observe fire().
/// Uses the real 3-frame consensus logic from Trigger::update().
class TestTrigger : public autotrigger::Trigger {
 public:
  bool inject_held_ = false;
  bool last_fire_   = false;
  int  fire_call_count_ = 0;

  bool is_held() override { return inject_held_; }
  void fire(bool on) override {
    last_fire_ = on;
    fire_call_count_++;
  }
};

// ============================================================================
// Helpers
// ============================================================================

static DetectBox MakeDetect(float cx, float cy, float w, float h,
                             float conf = 0.9f,
                             int   cls  = 0) {
  return {cx, cy, w, h, conf, cls};
}

/// 100 bytes of dummy RGB data — sufficient to satisfy the YOLO interface.
static const uint8_t kDummyRGB[100] = {};

// ============================================================================
// Fixture — assembles the full pipeline once for all tests
// ============================================================================

class PipelineTest : public ::testing::Test {
 protected:
  void SetUp() override {
    yolo_.set_detections({});
    yolo_.init("mock_model");  // sets ready_ = true

    // Feed a valid ranging frame so is_healthy() → true, is_safe() → true
    ranging_.set_uart_read_fn([](uint8_t* buf, size_t) {
      // JRT TOF01: 8-byte frame → 10.0 m = 1000 cm = 0x03E8
      buf[0] = 0x5A;  buf[1] = 0xE8;  buf[2] = 0x03;
      buf[3] = 0x00;  buf[4] = 0x00;  buf[5] = 0x00;
      buf[6] = 0x00;
      buf[7] = static_cast<uint8_t>(0x5A + 0xE8 + 0x03);
      return 8;
    });
    ranging_.get_distance();  // consume frame → has_valid_frame_ = true

    // SafetyMock with nullptr camera — kick_watchdog() never touches it.
    safety_ = new autotrigger::SafetyMock(nullptr, &ranging_, &trigger_);
    safety_->do_startup_check();  // sets startup_ok_ → is_safe() returns true
    safety_->kick_watchdog();     // initialize watchdog so is_safe() passes
    trigger_.fire_call_count_ = 0;  // reset: do_startup_check() fires once

    pipeline_ = new autotrigger::Pipeline(
        &yolo_, &kalman_, &ballistics_,
        &ranging_, &display_, &trigger_, safety_);
  }

  void TearDown() override {
    delete pipeline_;
    delete safety_;
  }

  // Modules
  YOLOMock    yolo_;
  ::KalmanFilter           kalman_;
  autotrigger::Ballistics  ballistics_;
  autotrigger::Ranging     ranging_{"/dev/ttyS1"};
  autotrigger::DisplayMock display_;
  TestTrigger              trigger_;

  autotrigger::SafetyMock* safety_   = nullptr;
  autotrigger::Pipeline*   pipeline_ = nullptr;
};

// ============================================================================
// Test 1 — Detection at centre → aimpoint moves above centre (drop compensation)
// ============================================================================

TEST_F(PipelineTest, DetectionAtCentreDropApplied) {
  // bbox h = 0.15 → 96 px → monocular distance ≈ 530×1.7/96 ≈ 9.4 m
  // At 9.4 m the drop is non-trivial, pushing aim_y above crosshair (y < 120).
  yolo_.set_detections({MakeDetect(0.50f, 0.50f, 0.20f, 0.15f)});

  pipeline_->run_frame(kDummyRGB);

  int ax = 0, ay = 0;
  pipeline_->get_aimpoint(ax, ay);

  // Stationary centred target → no lead, so aim_x should stay at crosshair
  EXPECT_EQ(ax, 120);

  // Drop compensation moves aimpoint ABOVE centre on display (smaller y).
  // drop_pixels > 0 means "aim above target" → aim_y < CROSSHAIR_Y.
  EXPECT_LT(ay, 120);
}

// ============================================================================
// Test 2 — No detection → coast → lost → aimpoint returns to centre
// ============================================================================

TEST_F(PipelineTest, NoDetectionCoastReturnsToCentre) {
  // No detections at all
  yolo_.set_detections({});

  // Run enough frames to exhaust the Kalman coast allowance (3 misses).
  for (int i = 0; i < 5; ++i) {
    pipeline_->run_frame(kDummyRGB);
  }

  int ax = 0, ay = 0;
  pipeline_->get_aimpoint(ax, ay);

  EXPECT_EQ(ax, 120);
  EXPECT_EQ(ay, 120);
  EXPECT_FALSE(pipeline_->is_aim_aligned());
}

// ============================================================================
// Test 3 — Moving target → lead pixels applied to aim_x
// ============================================================================

TEST_F(PipelineTest, MovingTargetLeadApplied) {
  // Feed a detection that moves rightward across several frames so
  // the Kalman builds a non-zero velocity estimate.
  yolo_.set_detections({MakeDetect(0.40f, 0.50f, 0.20f, 0.15f)});
  pipeline_->run_frame(kDummyRGB);

  yolo_.set_detections({MakeDetect(0.55f, 0.50f, 0.20f, 0.15f)});
  pipeline_->run_frame(kDummyRGB);

  yolo_.set_detections({MakeDetect(0.70f, 0.50f, 0.20f, 0.15f)});
  pipeline_->run_frame(kDummyRGB);

  int ax = 0, ay = 0;
  pipeline_->get_aimpoint(ax, ay);

  // Rightward velocity → positive lead → aim_x > crosshair centre
  EXPECT_GT(ax, 120);
}

// ============================================================================
// Test 4 — Aim aligned → trigger fires after 3-frame consensus
// ============================================================================

TEST_F(PipelineTest, AimAlignedTriggersFire) {
  trigger_.inject_held_ = true;

  // h=0.117 → 75px bbox → range ≈ 530·1.7/75 ≈ 12m.
  // At 12m: bore offset ≈ 0, drop minimal → aimpoint stays within 8px threshold.
  yolo_.set_detections({MakeDetect(0.50f, 0.50f, 0.20f, 0.117f)});

  // Frame 1 — consensus_counter = 1, fire(false)
  pipeline_->run_frame(kDummyRGB);
  EXPECT_FALSE(trigger_.last_fire_);
  EXPECT_EQ(trigger_.fire_call_count_, 1);

  // Frame 2 — consensus_counter = 2, fire(false)
  pipeline_->run_frame(kDummyRGB);
  EXPECT_FALSE(trigger_.last_fire_);
  EXPECT_EQ(trigger_.fire_call_count_, 2);

  // Frame 3 — consensus_counter = 3 → fire(true) !
  pipeline_->run_frame(kDummyRGB);
  EXPECT_TRUE(trigger_.last_fire_);
  EXPECT_EQ(trigger_.fire_call_count_, 3);
}

// ============================================================================
// Test 5 — Aim NOT aligned → trigger stays LOW
// ============================================================================

TEST_F(PipelineTest, AimNotAlignedTriggerStaysLow) {
  trigger_.inject_held_ = true;

  // Tiny bbox → far range. With low v0=15 m/s → enormous drop → aim_y ≪ 120
  pipeline_->set_muzzle_velocity(15.0f);
  yolo_.set_detections({MakeDetect(0.50f, 0.50f, 0.05f, 0.03125f)});

  for (int i = 0; i < 10; ++i) {
    pipeline_->run_frame(kDummyRGB);
  }

  EXPECT_FALSE(trigger_.last_fire_);
  EXPECT_GT(trigger_.fire_call_count_, 0);

  int ax = 0, ay = 0;
  pipeline_->get_aimpoint(ax, ay);
  EXPECT_TRUE(ax != 120 || ay != 120);
}

// ============================================================================
// Test 6 — Aim re-acquired after loss fires again
// ============================================================================

TEST_F(PipelineTest, AimReacquiredAfterLossFiresAgain) {
  trigger_.inject_held_ = true;

  // Phase 1: lock and fire (h=0.117 → ~12m, bore near zero → aligned)
  yolo_.set_detections({MakeDetect(0.50f, 0.50f, 0.20f, 0.117f)});
  for (int i = 0; i < 3; ++i) pipeline_->run_frame(kDummyRGB);
  ASSERT_TRUE(trigger_.last_fire_);

  // Phase 2: lose the target completely
  yolo_.set_detections({});
  for (int i = 0; i < 5; ++i) pipeline_->run_frame(kDummyRGB);
  ASSERT_FALSE(trigger_.last_fire_);

  // Phase 3: re-acquire — need 3 new consensus frames (count resets on loss)
  yolo_.set_detections({MakeDetect(0.50f, 0.50f, 0.20f, 0.117f)});
  
  // R1: kalman re-initializes, consensus = 1
  pipeline_->run_frame(kDummyRGB);
  EXPECT_FALSE(trigger_.last_fire_);

  // R2: kalman converges, consensus = 2
  pipeline_->run_frame(kDummyRGB);
  EXPECT_FALSE(trigger_.last_fire_);

  // R3: consensus = 3 → fire!
  pipeline_->run_frame(kDummyRGB);
  ASSERT_TRUE(trigger_.last_fire_);
}

// ============================================================================
// Test 7 — Trigger NOT held → never fires regardless of alignment
// ============================================================================

TEST_F(PipelineTest, TriggerNotHeldNeverFires) {
  trigger_.inject_held_ = false;  // physical trigger released

  yolo_.set_detections({MakeDetect(0.50f, 0.50f, 0.60f, 0.75f)});

  for (int i = 0; i < 10; ++i) {
    pipeline_->run_frame(kDummyRGB);
  }

  EXPECT_FALSE(trigger_.last_fire_);
}
