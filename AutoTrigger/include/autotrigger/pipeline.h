#pragma once

#include "autotrigger/ballistics.h"
#include "autotrigger/hal/idisplay.h"
#include "autotrigger/kalman_filter.h"
#include "autotrigger/ranging.h"
#include "autotrigger/safety.h"
#include "autotrigger/trigger.h"
#include "autotrigger/yolo_infer.h"

namespace autotrigger {

/// Pipeline — integrates YOLO → Kalman → Ranging → Ballistics → Trigger → Display
///
/// Each call to run_frame() executes one complete cycle (~33 ms at 30 FPS):
///   1. YOLO inference on the RGB frame
///   2. Convert normalised DetectBox to pixel-space BoundingBox
///   3. Kalman predict + update (or coast on missed detection)
///   4. Monocular ranging via set_monocular_input()
///   5. Ballistics compute_aimpoint() → drop pixels + time-of-flight
///   6. Lead compensation from Kalman velocity estimate
///   7. Alignment check: |aim − crosshair| < ALIGN_THRESHOLD
///   8. Trigger consensus update
///   9. Display render + safety watchdog kick
///
/// After 3 consecutive missed detections the Kalman declares the target lost
/// and the aimpoint returns to the crosshair centre (120, 120).
class Pipeline {
 public:
  Pipeline(::YOLOInfer* yolo, ::KalmanFilter* kf, Ballistics* bal,
           Ranging* ranging, IDisplay* display, Trigger* trigger,
           Safety* safety);

  /// Returns true when YOLO is ready.
  bool init();

  /// Execute one complete pipeline frame.
  /// @param rgb_data  640×640×3 RGB UINT8 image (NHWC layout), may be nullptr.
  void run_frame(const uint8_t* rgb_data);

  /// True when the computed aimpoint lies within ALIGN_THRESHOLD of the
  /// crosshair centre (120, 120) in display coordinates.
  bool is_aim_aligned() const { return aim_aligned_; }

  /// Current aimpoint on the 240×240 display.
  void get_aimpoint(int& x, int& y) const {
    x = aim_x_;
    y = aim_y_;
  }

  // ── Tuning knobs (all pixel-space values at 640×640 nominal resolution) ──

  /// Override the default muzzle velocity (70 m/s).
  void set_muzzle_velocity(float v0) { v0_ = v0; }
  float muzzle_velocity() const { return v0_; }
  /// Bore height: camera sensor centre above barrel bore axis (metres).
  float bore_height() const { return BORE_HEIGHT; }

 private:
  // ── Module pointers ──────────────────────────────────────────────────
  ::YOLOInfer*   yolo_;
  ::KalmanFilter* kalman_;
  Ballistics*     ballistics_;
  Ranging*        ranging_;
  IDisplay*       display_;
  Trigger*        trigger_;
  Safety*         safety_;

  // ── Constants ────────────────────────────────────────────────────────
  static constexpr float FOCAL          = 530.0f;   ///< 62° HFOV @ 640 px
  static constexpr int   SCREEN_W       = 640;
  static constexpr int   SCREEN_H       = 640;
  static constexpr int   DISPLAY_W      = 240;
  static constexpr int   DISPLAY_H      = 240;
  static constexpr int   CROSSHAIR_X    = 120;      ///< display coords
  static constexpr int   CROSSHAIR_Y    = 120;
  static constexpr int   ALIGN_THRESHOLD = 8;       ///< pixels (display)
  static constexpr float DEFAULT_V0     = 70.0f;    ///< m/s
  static constexpr float DT             = 0.033f;   ///< 1/30 s
  static constexpr float BORE_HEIGHT    = 0.10f;    ///< camera above bore (m)
  static constexpr float BORE_REF_DIST  = 15.0f;    ///< boresight ref distance (m)

  // ── Adjustable parameters ────────────────────────────────────────────
  float v0_          = DEFAULT_V0;

  // ── State ────────────────────────────────────────────────────────────
  int  aim_x_        = CROSSHAIR_X;
  int  aim_y_        = CROSSHAIR_Y;
  bool aim_aligned_  = false;
  bool target_locked_ = false;

  /// Convert a normalised YOLO DetectBox to pixel-space BoundingBox.
  static ::BoundingBox to_pixel_bbox(const DetectBox& det);
};

}  // namespace autotrigger
