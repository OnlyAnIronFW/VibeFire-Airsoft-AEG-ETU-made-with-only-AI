#pragma once

#include <Eigen/Dense>

/// Bounding-box detection from YOLO or any object detector.
struct BoundingBox {
  float cx;  // center x (pixels)
  float cy;  // center y (pixels)
  float w;   // width (pixels)
  float h;   // height (pixels)
};

/// 8-DOF Kalman filter for single-target tracking.
///
/// State vector (8):
///   x = [cx, cy, vx, vy, ax, ay, w, h]ᵀ
///
/// Measurement (4):
///   z = [cx, cy, w, h]ᵀ
///
/// Process model: constant acceleration for position/velocity/accel;
/// constant (identity) for bounding-box dimensions.
///
/// Measurement noise R is adaptive: R = R_base · (h_ref / h)²
/// so that small (far) detections receive higher measurement uncertainty.
class KalmanFilter {
 public:
  static constexpr int kStateDim = 8;   // [cx, cy, vx, vy, ax, ay, w, h]
  static constexpr int kMeasDim = 4;    // [cx, cy, w, h]

  static constexpr int kMaxConsecutiveMisses = 3;
  static constexpr float kRefHeightPx = 160.0f;   // ~1.7m @ 10m, typical
  static constexpr float kHeightDeviationThreshold = 0.30f;

  using StateVec  = Eigen::Matrix<float, kStateDim, 1>;
  using StateMat  = Eigen::Matrix<float, kStateDim, kStateDim>;
  using GainMat   = Eigen::Matrix<float, kStateDim, kMeasDim>;  // 8x4
  using MeasState = Eigen::Matrix<float, kMeasDim, kStateDim>;  // 4x8 (H)
  using MeasVec   = Eigen::Matrix<float, kMeasDim, 1>;
  using MeasMat   = Eigen::Matrix<float, kMeasDim, kMeasDim>;

  KalmanFilter();

  // --- lifecycle -------------------------------------------------------
  void init(const BoundingBox& det);
  void predict(float dt, bool count_miss = true);
  void update(const BoundingBox& det);
  void coast();  // predict-only for one missed frame

  /// Project state N steps ahead WITHOUT mutating this filter.
  /// Used for latency-compensated aimpoint computation.
  StateVec predict_ahead(int n, float dt) const;

  // --- queries ---------------------------------------------------------
  StateVec get_state() const;
  StateMat get_covariance() const;
  float mahalanobis_distance(const BoundingBox& det) const;
  bool is_lost() const;
  bool is_height_consistent() const;
  int consecutive_misses() const { return miss_count_; }
  float get_measurement_noise_trace() const;
  float get_prediction_noise_trace() const;

 private:
  void build_F(float dt);
  void build_Q(float dt);
  void update_R_from_measurement(float height);

  // Kalman matrices
  StateVec  x_;   // state estimate
  StateMat  P_;   // error covariance
  StateMat  F_;   // state transition
  MeasState H_;   // measurement matrix
  StateMat  Q_;   // process noise
  MeasMat   R_;   // measurement noise (adaptive)

  // Book-keeping
  int   miss_count_{0};
  float last_dt_{0.033f};
  float initial_height_{0.0f};
  float last_measured_height_{0.0f};

  // Noise base parameters
  StateMat base_Q_diag_;   // per-axis acceleration variance
  MeasMat  base_R_diag_;   // per-measurement variance at ref height
};
