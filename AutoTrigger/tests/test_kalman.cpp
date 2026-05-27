#include "autotrigger/kalman_filter.h"
#include <Eigen/Dense>
#include <gtest/gtest.h>

// ---------------------------------------------------------------------------
// Test 1: Initialization — state set from detection, velocity/accel zero
// ---------------------------------------------------------------------------
TEST(KalmanFilterTest, InitializeWithZeroVelocity) {
  KalmanFilter kf;
  BoundingBox det = {320.0f, 240.0f, 100.0f, 200.0f};  // cx, cy, w, h
  kf.init(det);
  auto state = kf.get_state();
  EXPECT_NEAR(state(0), 320.0f, 1e-4f);  // cx
  EXPECT_NEAR(state(1), 240.0f, 1e-4f);  // cy
  EXPECT_NEAR(state(6), 100.0f, 1e-4f);  // w
  EXPECT_NEAR(state(7), 200.0f, 1e-4f);  // h
  // Velocity/acceleration start at 0
  EXPECT_NEAR(state(2), 0.0f, 1e-4f);
  EXPECT_NEAR(state(3), 0.0f, 1e-4f);
}

// ---------------------------------------------------------------------------
// Test 2: Predict-only → velocity persists, covariance grows
// ---------------------------------------------------------------------------
TEST(KalmanFilterTest, PredictGrowsCovariance) {
  KalmanFilter kf;
  kf.init({320, 240, 100, 200});
  auto cov_before = kf.get_covariance().trace();
  kf.predict(0.033f);  // 33 ms = 1 frame at 30 FPS
  auto cov_after = kf.get_covariance().trace();
  EXPECT_GT(cov_after, cov_before);  // Uncertainty grows without measurement
}

// ---------------------------------------------------------------------------
// Test 3: Measurement update → covariance shrinks
// ---------------------------------------------------------------------------
TEST(KalmanFilterTest, UpdateReducesCovariance) {
  KalmanFilter kf;
  kf.init({320, 240, 100, 200});
  kf.predict(0.033f);
  auto cov_before = kf.get_covariance().trace();
  kf.update({322, 241, 99, 201});  // Near match
  auto cov_after = kf.get_covariance().trace();
  EXPECT_LT(cov_after, cov_before);
}

// ---------------------------------------------------------------------------
// Test 4: Moving target tracking (rightward at ~150 px/s)
// ---------------------------------------------------------------------------
TEST(KalmanFilterTest, TrackingMovingTarget) {
  KalmanFilter kf;
  kf.init({100, 240, 100, 200});
  // Simulate 30 frames of target moving right at 5 px/frame
  for (int i = 0; i < 30; i++) {
    kf.predict(0.033f);
    kf.update({100.0f + 5.0f * static_cast<float>(i), 240.0f, 100.0f, 200.0f});
  }
  auto state = kf.get_state();
  // vx ≈ 150 px/s (5 px/frame × 30 fps)
  EXPECT_NEAR(state(2), 150.0f, 20.0f);
}

// ---------------------------------------------------------------------------
// Test 5: Coast on missed detection (1-3 frames tolerated)
// ---------------------------------------------------------------------------
TEST(KalmanFilterTest, CoastOnMiss) {
  KalmanFilter kf;
  kf.init({320, 240, 100, 200});
  // Track for 10 frames
  for (int i = 0; i < 10; i++) {
    kf.predict(0.033f);
    kf.update({320.0f + static_cast<float>(i), 240.0f, 100.0f, 200.0f});
  }
  // Miss 2 frames
  kf.predict(0.033f);
  kf.predict(0.033f);
  EXPECT_FALSE(kf.is_lost());  // Still tracking
  // Miss 4th frame
  kf.predict(0.033f);
  EXPECT_TRUE(kf.is_lost());  // Lost after 3 consecutive misses
}

// ---------------------------------------------------------------------------
// Test 6: Adaptive measurement noise R ∝ 1/h²
// ---------------------------------------------------------------------------
TEST(KalmanFilterTest, AdaptiveNoiseByBboxSize) {
  KalmanFilter kf1, kf2;
  kf1.init({320, 240, 100, 200});  // Large bbox (close target)
  kf2.init({320, 240, 20, 40});    // Small bbox (far target)
  // Far target should have higher measurement noise
  // R scales: R_far ≈ R_close · (h_close/h_far)² = 25×
  float r1 = kf1.get_measurement_noise_trace();
  float r2 = kf2.get_measurement_noise_trace();
  EXPECT_GT(r2, r1 * 4.0f);
}

// ---------------------------------------------------------------------------
// Test 7: Mahalanobis distance for data association
// ---------------------------------------------------------------------------
TEST(KalmanFilterTest, MahalanobisAssociation) {
  KalmanFilter kf;
  kf.init({320, 240, 100, 200});
  kf.predict(0.033f);
  // Detection very close → should associate
  float d1 = kf.mahalanobis_distance({322, 241, 99, 201});
  // Detection very far → should reject
  float d2 = kf.mahalanobis_distance({500, 400, 50, 50});
  EXPECT_LT(d1, 10.0f);   // Close match
  EXPECT_GT(d2, 50.0f);   // Far → high distance → reject
}

// ---------------------------------------------------------------------------
// Test 8: Bbox height deviation triggers low confidence
// ---------------------------------------------------------------------------
TEST(KalmanFilterTest, HeightDeviationConfidenceFlag) {
  KalmanFilter kf;
  kf.init({320, 240, 100, 200});  // h = 200
  kf.predict(0.033f);
  kf.update({322, 241, 100, 200});  // Normal
  EXPECT_TRUE(kf.is_height_consistent());
  kf.predict(0.033f);
  kf.update({322, 241, 100, 100});  // h halved → crouching/prone
  EXPECT_FALSE(kf.is_height_consistent());
}
