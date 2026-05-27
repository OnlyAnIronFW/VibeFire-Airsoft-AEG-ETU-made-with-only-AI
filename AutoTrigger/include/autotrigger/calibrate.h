#pragma once

#include <string>

namespace autotrigger {

/// Camera intrinsic calibration data.
struct CameraCalibration {
    float fx = 530.0f;   // focal length x (pixels)
    float fy = 530.0f;   // focal length y (pixels)
    float cx = 320.0f;   // principal point x (pixels)
    float cy = 320.0f;   // principal point y (pixels)
    float k1 = 0.0f;     // radial distortion
    float k2 = 0.0f;
    float p1 = 0.0f;     // tangential distortion
    float p2 = 0.0f;

    bool load(const std::string& path);
    bool save(const std::string& path) const;
};

/// Boresight alignment data.
struct BoreCalibration {
    float offset_px = 0.0f;         // vertical pixel offset at ref distance
    float reference_distance = 15.0f; // meters
    float camera_height = 0.10f;    // camera above bore (meters)

    bool load(const std::string& path);
    bool save(const std::string& path) const;
};

/// Ballistic calibration data.
struct BallisticCalibration {
    float muzzle_velocity = 70.0f; // m/s
    float drag_coefficient = 0.67f;
    float dart_mass = 0.002f;     // kg (2g)

    bool load(const std::string& path);
    bool save(const std::string& path) const;
};

/// Calibration procedure (static methods).
/// Run once per gun configuration (~5 minutes).
class Calibration {
public:
    /// Calibrate camera intrinsics using a checkerboard.
    static bool calibrate_camera(const std::string& image_dir,
                                  CameraCalibration& calib);

    /// Calibrate boresight offset at known distance.
    static bool calibrate_boresight(float distance_m,
                                     float pixel_offset_y,
                                     BoreCalibration& calib);

    /// Calibrate muzzle velocity from 10-shot group.
    static bool calibrate_ballistics(float drop_meters,
                                      float range_meters,
                                      BallisticCalibration& calib);
};

} // namespace autotrigger
