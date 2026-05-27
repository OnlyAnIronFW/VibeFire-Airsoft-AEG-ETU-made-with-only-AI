#include "autotrigger/kalman_filter.h"

#include <cmath>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
KalmanFilter::KalmanFilter() {
  // -- measurement matrix H_ (constant) ---------------------------------
  H_.setZero();
  H_(0, 0) = 1.0f;   // z_cx = cx
  H_(1, 1) = 1.0f;   // z_cy = cy
  H_(2, 6) = 1.0f;   // z_w  = w
  H_(3, 7) = 1.0f;   // z_h  = h

  // -- base measurement noise (at reference height h_ref = 160 px) -----
  base_R_diag_.setIdentity();
  base_R_diag_ *= 1.0f;   // 1 px^2 variance per measurement channel

  // -- process noise: sigma_a = 50 px/s^2 -> q = 2500 -----------------
  // build_Q() computes the discrete Singer-model components.
}

// ---------------------------------------------------------------------------
// init -- seed the filter with a first detection
// ---------------------------------------------------------------------------
void KalmanFilter::init(const BoundingBox& det) {
  // State vector: [cx, cy, vx, vy, ax, ay, w, h]
  x_.setZero();
  x_(0) = det.cx;
  x_(1) = det.cy;
  x_(6) = det.w;
  x_(7) = det.h;

  // Initial covariance: confident about position & bbox,
  // gradually more uncertain about velocity & acceleration.
  P_.setIdentity();
  P_(0, 0) = 1.0f;    // cx:  1 px std
  P_(1, 1) = 1.0f;    // cy
  P_(2, 2) = 10.0f;   // vx:  3.2 px/s std
  P_(3, 3) = 10.0f;   // vy
  P_(4, 4) = 100.0f;  // ax: 10 px/s^2 std
  P_(5, 5) = 100.0f;  // ay
  P_(6, 6) = 4.0f;    // w:   2 px std
  P_(7, 7) = 4.0f;    // h

  // Book-keeping
  miss_count_           = 0;
  last_dt_              = 0.033f;
  initial_height_       = det.h;
  last_measured_height_ = det.h;

  // Adaptive measurement noise from initial detection height
  update_R_from_measurement(det.h);

  // F_, Q_ default to identity / zero; rebuilt in predict().
  F_.setIdentity();
  Q_.setZero();
}

// ---------------------------------------------------------------------------
// predict -- advance state & covariance by dt seconds
// ---------------------------------------------------------------------------
void KalmanFilter::predict(float dt, bool count_miss) {
  last_dt_ = dt;
  build_F(dt);
  build_Q(dt);

  // State prediction
  x_ = F_ * x_;

  // Covariance prediction
  P_ = F_ * P_ * F_.transpose() + Q_;

  // Only count as missed frame for tracking (not for latency projection)
  if (count_miss) {
    miss_count_++;
  }
}

// ---------------------------------------------------------------------------
// update -- fuse a detection measurement
// ---------------------------------------------------------------------------
void KalmanFilter::update(const BoundingBox& det) {
  // Adaptive measurement noise
  update_R_from_measurement(det.h);

  // Innovation covariance: S = H*P*H^T + R
  GainMat PHT = P_ * H_.transpose();    // 8x8 * 8x4 = 8x4
  MeasMat S   = H_ * PHT + R_;          // 4x8 * 8x4 = 4x4

  // Kalman gain: K = P*H^T*S^{-1}  (8x4 matrix)
  GainMat K = PHT * S.inverse();

  // Measurement vector
  MeasVec z;
  z << det.cx, det.cy, det.w, det.h;

  // State update: x = x + K*(z - H*x)
  x_ = x_ + K * (z - H_ * x_);

  // Covariance update: P = (I - K*H)*P  (Joseph form)
  StateMat I = StateMat::Identity();
  P_ = (I - K * H_) * P_;

  // Book-keeping
  miss_count_            = 0;
  last_measured_height_  = det.h;
}

// ---------------------------------------------------------------------------
// coast -- one frame without measurement
// ---------------------------------------------------------------------------
void KalmanFilter::coast() {
  predict(last_dt_);
}

// ---------------------------------------------------------------------------
// mahalanobis_distance -- gate detections for data association
// ---------------------------------------------------------------------------
float KalmanFilter::mahalanobis_distance(const BoundingBox& det) const {
  MeasVec z;
  z << det.cx, det.cy, det.w, det.h;

  MeasVec y = z - H_ * x_;                      // innovation
  MeasMat S = H_ * P_ * H_.transpose() + R_;    // innovation covariance

  // d^2 = y^T * S^{-1} * y
  float d2 = y.transpose() * S.inverse() * y;
  return std::sqrt(d2);
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------
KalmanFilter::StateVec KalmanFilter::get_state() const {
  return x_;
}

KalmanFilter::StateMat KalmanFilter::get_covariance() const {
  return P_;
}

bool KalmanFilter::is_lost() const {
  return miss_count_ >= kMaxConsecutiveMisses;
}

bool KalmanFilter::is_height_consistent() const {
  if (initial_height_ <= 0.0f) return true;
  float ratio = std::abs(last_measured_height_ - initial_height_) / initial_height_;
  return ratio < kHeightDeviationThreshold;
}

float KalmanFilter::get_measurement_noise_trace() const {
  return R_.trace();
}

float KalmanFilter::get_prediction_noise_trace() const {
  return Q_.trace();
}

// =========================================================================
// Private helpers
// =========================================================================

// ---------------------------------------------------------------------------
// build_F -- state-transition matrix for given dt
//
// Constant-acceleration model for (cx, vx, ax) and (cy, vy, ay);
// identity for (w, h).
// ---------------------------------------------------------------------------
void KalmanFilter::build_F(float dt) {
  F_.setIdentity();

  float dt2_2 = dt * dt * 0.5f;   // dt^2 / 2

  // cx row
  F_(0, 2) = dt;
  F_(0, 4) = dt2_2;

  // cy row
  F_(1, 3) = dt;
  F_(1, 5) = dt2_2;

  // vx row: identity for vx, plus dt for ax
  F_(2, 4) = dt;

  // vy row: identity for vy, plus dt for ay
  F_(3, 5) = dt;

  // ax, ay, w, h -- identity (no deterministic change)
}

// ---------------------------------------------------------------------------
// build_Q -- process-noise covariance matrix
//
// Singer model: piecewise-constant white acceleration noise with spectral
// density q = sigma_a^2 (acceleration variance).
//
// For each (pos, vel, acc) triplet:
//   Q_pp = q*dt^5/20    Q_pv = q*dt^4/8     Q_pa = q*dt^3/6
//   Q_vp = q*dt^4/8     Q_vv = q*dt^3/3     Q_va = q*dt^2/2
//   Q_ap = q*dt^3/6     Q_av = q*dt^2/2     Q_aa = q*dt
//
// Bbox dimensions get negligible process noise.
// ---------------------------------------------------------------------------
void KalmanFilter::build_Q(float dt) {
  Q_.setZero();

  // Acceleration noise intensity
  constexpr float sigma_a = 50.0f;              // px/s^2
  constexpr float q = sigma_a * sigma_a;        // 2500 px^2/s^4

  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;
  float dt5 = dt4 * dt;

  // -- (cx, vx, ax) block -- indices 0, 2, 4 --
  Q_(0, 0) = q * dt5 / 20.0f;
  Q_(0, 2) = Q_(2, 0) = q * dt4 / 8.0f;
  Q_(0, 4) = Q_(4, 0) = q * dt3 / 6.0f;
  Q_(2, 2) = q * dt3 / 3.0f;
  Q_(2, 4) = Q_(4, 2) = q * dt2 / 2.0f;
  Q_(4, 4) = q * dt;

  // -- (cy, vy, ay) block -- indices 1, 3, 5 --
  Q_(1, 1) = q * dt5 / 20.0f;
  Q_(1, 3) = Q_(3, 1) = q * dt4 / 8.0f;
  Q_(1, 5) = Q_(5, 1) = q * dt3 / 6.0f;
  Q_(3, 3) = q * dt3 / 3.0f;
  Q_(3, 5) = Q_(5, 3) = q * dt2 / 2.0f;
  Q_(5, 5) = q * dt;

  // -- bbox (w, h) -- indices 6, 7 -- tiny process noise --
  constexpr float sigma_b_sq = 0.01f;   // 0.1 px std
  Q_(6, 6) = sigma_b_sq;
  Q_(7, 7) = sigma_b_sq;
}

// ---------------------------------------------------------------------------
// update_R_from_measurement -- adaptive R proportional to 1/h^2
//
// R = R_base * (h_ref / h)^2
//
// Far targets (small h) -> larger measurement noise;
// near targets (large h) -> smaller measurement noise.
// ---------------------------------------------------------------------------
void KalmanFilter::update_R_from_measurement(float height) {
  if (height < 1.0f) height = 1.0f;            // safety clamp
  float scale = (kRefHeightPx / height);
  scale *= scale;                                // (h_ref / h)^2
  R_ = base_R_diag_ * scale;
}

// ---------------------------------------------------------------------------
// predict_ahead 鈥?const projection for latency compensation
//
// Computes F^n 路 x_ without mutating this filter.  F is rebuilt locally
// using the same constant-acceleration model as build_F().  Binary
// exponentiation keeps this O(log n) for small n (typically 2鈥?).
// ---------------------------------------------------------------------------
KalmanFilter::StateVec KalmanFilter::predict_ahead(int n, float dt) const {
  if (n <= 0) return x_;

  // Build 1-step transition matrix (mirrors build_F)
  StateMat F = StateMat::Identity();
  float dt2_2 = dt * dt * 0.5f;
  F(0, 2) = dt;    F(0, 4) = dt2_2;   // cx 鈫?vx路dt + ax路dt虏/2
  F(1, 3) = dt;    F(1, 5) = dt2_2;   // cy 鈫?vy路dt + ay路dt虏/2
  F(2, 4) = dt;                        // vx 鈫?ax路dt
  F(3, 5) = dt;                        // vy 鈫?ay路dt
  // w, h rows remain identity

  // Compute F^n via binary exponentiation
  StateMat Fn  = StateMat::Identity();
  StateMat Fp  = F;
  int      rem = n;
  while (rem > 0) {
    if (rem & 1) Fn = Fn * Fp;
    Fp = Fp * Fp;
    rem >>= 1;
  }
  return Fn * x_;
}
