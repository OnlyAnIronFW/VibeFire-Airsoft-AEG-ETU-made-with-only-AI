#include "autotrigger/calibrate.h"
#include <fstream>
#include <iostream>
#include <cmath>

namespace autotrigger {

// ── CameraCalibration ──────────────────────────

bool CameraCalibration::load(const std::string& path) {
    std::ifstream f(path);
    if (!f) return false;
    f >> fx >> fy >> cx >> cy >> k1 >> k2 >> p1 >> p2;
    return f.good();
}

bool CameraCalibration::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f) return false;
    f << fx << " " << fy << " " << cx << " " << cy << " "
      << k1 << " " << k2 << " " << p1 << " " << p2 << "\n";
    return f.good();
}

// ── BoreCalibration ────────────────────────────

bool BoreCalibration::load(const std::string& path) {
    std::ifstream f(path);
    if (!f) return false;
    f >> offset_px >> reference_distance >> camera_height;
    return f.good();
}

bool BoreCalibration::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f) return false;
    f << offset_px << " " << reference_distance << " "
      << camera_height << "\n";
    return f.good();
}

// ── BallisticCalibration ───────────────────────

bool BallisticCalibration::load(const std::string& path) {
    std::ifstream f(path);
    if (!f) return false;
    f >> muzzle_velocity >> drag_coefficient >> dart_mass;
    return f.good();
}

bool BallisticCalibration::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f) return false;
    f << muzzle_velocity << " " << drag_coefficient << " "
      << dart_mass << "\n";
    return f.good();
}

// ── Calibration Procedures ─────────────────────

bool Calibration::calibrate_camera(const std::string& image_dir,
                                    CameraCalibration& calib) {
    // Uses OpenCV calibrateCamera() with checkerboard images.
    // On x86 (no OpenCV): returns defaults.
#ifdef MOCK_MODE
    (void)image_dir;
    std::cout << "[Calibrate] Camera: using defaults (fx="
              << calib.fx << ", fy=" << calib.fy << ")\n";
    return true;
#else
    // TODO: OpenCV checkerboard calibration
    // cv::glob(image_dir + "/*.jpg", images);
    // cv::calibrateCamera(object_points, image_points, size, K, D, rvecs, tvecs);
    std::cerr << "Camera calibration requires OpenCV (not available)\n";
    return false;
#endif
}

bool Calibration::calibrate_boresight(float distance_m,
                                       float pixel_offset_y,
                                       BoreCalibration& calib) {
    calib.offset_px = pixel_offset_y;
    calib.reference_distance = distance_m;
    calib.camera_height = 0.10f; // default: ~10cm above bore
    std::cout << "[Calibrate] Boresight: " << pixel_offset_y
              << " px offset at " << distance_m << "m\n";
    return true;
}

bool Calibration::calibrate_ballistics(float drop_meters,
                                        float range_meters,
                                        BallisticCalibration& calib) {
    // Solve for v0 given drop and range using quadratic drag
    // drop = 0.5 * g * t^2 where t = range / v_avg
    // For now, just set the measured v0
    float tof_estimate = std::sqrt(2.0f * drop_meters / 9.81f);
    float v0_estimate = range_meters / tof_estimate;
    calib.muzzle_velocity = v0_estimate;
    std::cout << "[Calibrate] Ballistics: v0 ≈ " << v0_estimate
              << " m/s (drop=" << drop_meters << "m at " << range_meters << "m)\n";
    return true;
}

} // namespace autotrigger
