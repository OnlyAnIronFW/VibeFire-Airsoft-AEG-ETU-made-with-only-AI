#include "autotrigger/pipeline.h"

#include <algorithm>
#include <cmath>

namespace autotrigger {

// ============================================================================
// Construction
// ============================================================================

Pipeline::Pipeline(::YOLOInfer* yolo, ::KalmanFilter* kf, Ballistics* bal,
                   Ranging* ranging, IDisplay* display, Trigger* trigger,
                   Safety* safety)
    : yolo_(yolo),
      kalman_(kf),
      ballistics_(bal),
      ranging_(ranging),
      display_(display),
      trigger_(trigger),
      safety_(safety) {}

bool Pipeline::init() { return yolo_->is_ready(); }

// ============================================================================
// to_pixel_bbox �?normalised [0-1] �?pixel space at 640×640
// ============================================================================

::BoundingBox Pipeline::to_pixel_bbox(const DetectBox& det) {
  ::BoundingBox b;
  b.cx = det.cx * SCREEN_W;
  b.cy = det.cy * SCREEN_H;
  b.w  = det.w  * SCREEN_W;
  b.h  = det.h  * SCREEN_H;
  return b;
}

// ============================================================================
// run_frame �?one complete pipeline cycle
// ============================================================================

void Pipeline::run_frame(const uint8_t* rgb_data) {
  // ── 1. YOLO inference ────────────────────────────────────────────────
  auto detections = yolo_->infer(rgb_data);

  // ── 2. Kalman tracking ───────────────────────────────────────────────
  if (!detections.empty()) {
    const auto& det = detections[0];  // primary (highest-confidence) target

    ::BoundingBox bbox = to_pixel_bbox(det);

    if (!target_locked_) {
      kalman_->init(bbox);
      target_locked_ = true;
    }

    kalman_->predict(DT);
    kalman_->update(bbox);
  } else {
    kalman_->coast();  // predict-only, increment miss counter
    if (kalman_->is_lost()) {
      target_locked_ = false;
    }
  }

  // ── 3. Aimpoint computation ──────────────────────────────────────────
  if (target_locked_ && !kalman_->is_lost()) {
    // Predict ahead for total pipeline latency (2 frames: YOLO + processing)
    // predict_ahead() is const �?projects state without polluting the filter
    auto state = kalman_->predict_ahead(2, DT);

    float pred_vx = state(2);   // velocity x (px/s)
    // vy available as state(3) if needed for vertical lead
    float pred_h  = state(7);   // predicted bbox height (pixels)

    // 3a. Monocular ranging
    ranging_->set_monocular_input(pred_h, FOCAL);
    float distance = ranging_->get_distance();  // metres

    // 3b. Ballistics drop
    AimpointResult aimpoint = ballistics_->compute_aimpoint(distance, v0_);

    // 3c. Scale from 640×640 sensor space �?240×240 display
    constexpr float scale = static_cast<float>(DISPLAY_W) / SCREEN_W;  // 0.375

    float drop_display = aimpoint.drop_pixels * scale;

    // Bore offset �?range-dependent per architecture plan §3.3:
    //   pixel_offset = fy · H_cam · (1 �?R/R_ref) / R
    // At R = R_ref (15 m): zero. At close range: positive. At long range: negative.
    float bore_640 = FOCAL * BORE_HEIGHT
                     * (1.0f - distance / BORE_REF_DIST)
                     / std::max(distance, 1.0f);
    float bore_display = bore_640 * scale;

    // Lead: velocity × time-of-flight �?pixel offset (640×640), then scale
    float lead_x_640     = pred_vx * aimpoint.time_of_flight;
    float lead_x_display = lead_x_640 * scale;

    aim_x_ = CROSSHAIR_X + static_cast<int>(lead_x_display);
    // drop_pixels = positive �?aim ABOVE target = smaller y in image coords
    aim_y_ = CROSSHAIR_Y - static_cast<int>(drop_display + bore_display);

    // Clamp to display bounds
    aim_x_ = std::max(0, std::min(DISPLAY_W - 1, aim_x_));
    aim_y_ = std::max(0, std::min(DISPLAY_H - 1, aim_y_));

    // 3d. Alignment check (in display pixels)
    aim_aligned_ = (std::abs(aim_x_ - CROSSHAIR_X) < ALIGN_THRESHOLD) &&
                   (std::abs(aim_y_ - CROSSHAIR_Y) < ALIGN_THRESHOLD);
  } else {
    // Target lost or never acquired �?return aimpoint to crosshair centre
    aim_x_       = CROSSHAIR_X;
    aim_y_       = CROSSHAIR_Y;
    aim_aligned_ = false;  // no valid firing solution without tracked target
  }

  // ── 4. Outputs (always executed) ─────────────────────────────────────
  display_->render(aim_x_, aim_y_);
  // Gate trigger on safety: prevents HIGH glitch when safety is violated
  if (safety_->is_safe()) {
    trigger_->update(aim_aligned_ && trigger_->is_held());
  } else {
    trigger_->update(false);  // fire(false) + reset consensus counter
  }
  // Watchdog kicked by main loop (avoids double-kick per frame)
}

}  // namespace autotrigger
