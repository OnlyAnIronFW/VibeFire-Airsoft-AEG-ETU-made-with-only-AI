#pragma once

#include <cstdint>
#include <string>

namespace autotrigger {

/**
 * @brief Result of a ballistics aim-point computation.
 *
 * drop_pixels and lead_x_pixels are offsets in image space. The caller
 * adds these to the detected bounding-box centre to obtain the final
 * aim-point coordinates.
 */
struct AimpointResult {
    /** Vertical pixel offset for the aiming point (positive = above target). */
    float drop_pixels = 0.0f;

    /** Horizontal lead offset in pixels (computed elsewhere via Kalman). */
    float lead_x_pixels = 0.0f;

    /** Estimated time of flight to the target, in seconds. */
    float time_of_flight = 0.0f;

    /** True when the requested range exceeds the maximum table entry. */
    bool out_of_range = false;
};

// ============================================================================
// BallisticsConfig — user-configurable physics parameters
// ============================================================================

struct BallisticsConfig {
    float dart_mass_kg     = 0.002f;        ///< Dart mass (kg), default 2g foam dart
    float drag_coefficient = 0.67f;         ///< Cd, flat-nose foam dart
    float dart_area_m2     = 1.13e-4f;      ///< Cross-section area (m²), 12mm ∅
    float air_density      = 1.225f;        ///< Air density (kg/m³) at sea level
    float gravity          = 9.81f;         ///< Gravitational acceleration (m/s²)
    float velocities[3]    = {55.0f, 70.0f, 90.0f}; ///< Muzzle velocity profiles (m/s)
};

/**
 * @brief Precomputed ballistics engine for foam-dart projectiles.
 *
 * Uses a bilinearly-interpolated lookup table indexed by (range, muzzle
 * velocity) to deliver sub-microsecond aim-point computation.  The table
 * is generated offline by `scripts/generate_drop_table.py`.
 *
 * Physics model: quadratic drag (flat-nose foam dart, 0.2 g, 12 mm ∅).
 * Default camera parameters assume a 62° HFOV lens on a 640×640 sensor.
 */
class Ballistics {
public:
    Ballistics();
    ~Ballistics();

    // ── Configuration ───────────────────────────

    /** Load a precomputed drop table from a binary file.
     *  Format: 801 × 3 float32 values (range-major, 0-80 m @ 0.1 m step).
     *  Returns false if the file cannot be opened. */
    bool load_table(const std::string& path);

    /** True after a successful load_table() call. */
    bool is_loaded() const;

    /** Set the camera focal length in pixels (default: 530). */
    void set_fy(float fy_px);

    /** Get the current focal length in pixels. */
    float fy() const;

    // ── Configurable physics ────────────────────

    /** Apply a custom physics configuration.
     *  Recalculates the internal alpha (drag) coefficient from the
     *  new parameters.  The drop table velocity columns are also
     *  updated — note that changing velocities requires the binary
     *  table file to have been generated with matching columns. */
    void set_config(const BallisticsConfig& cfg);

    /** Return the current physics configuration. */
    const BallisticsConfig& config() const;

    // ── Computation ─────────────────────────────

    /**
     * Compute the aim-point offset for a given range and muzzle velocity.
     *
     * @param range  Distance to target in metres (0–80 m).
     * @param v0     Muzzle velocity in m/s (typ. 55–90).
     * @return       AimpointResult with drop in pixels and time of flight.
     */
    AimpointResult compute_aimpoint(float range, float v0) const;

private:
    // ── Table layout ─────────────────────────────

    /// Table dimensions: RANGE_ENTRIES rows, VELOCITY_ENTRIES columns.
    static constexpr int kRangeEntries  = 801;   // 0–80 m @ 0.1 m
    static constexpr int kVelEntries    = 3;
    static constexpr float kRangeStep   = 0.1f;
    static constexpr float kMaxRange    = 80.0f;

    /// Muzzle-velocity profiles in the table (m/s).
    static constexpr float kVelocities[kVelEntries] = {55.0f, 70.0f, 90.0f};

    /// Precomputed drop values [range_index][velocity_index] → metres.
    float table_[kRangeEntries][kVelEntries] = {};

    bool loaded_ = false;

    // ── Camera parameters ───────────────────────

    float fy_ = 530.0f;   // pixels (62° HFOV @ 640 px)

    // ── Configurable physics ────────────────────

    BallisticsConfig config_{};   // user-configurable dart/atmosphere params
    float alpha_ = (1.225f * 1.13e-4f * 0.67f) / (2.0f * 0.002f); // default alpha
    float gravity_ = 9.81f;
    float velocities_[3] = {55.0f, 70.0f, 90.0f};

    // ── Physics constants (legacy, kept for reference) ──

    static constexpr float kGravity   = 9.81f;
    static constexpr float kRho       = 1.225f;      // air density (kg/m³)
    static constexpr float kDartArea  = 1.13e-4f;    // 12 mm ∅ cross-section
    static constexpr float kCd        = 0.67f;       // flat-nose foam dart
    static constexpr float kMass      = 0.002f;      // 2 g foam dart — consistent with generate_drop_table.py
                                                       // User spec: 0.2-0.5 g; verify actual dart mass
    static constexpr float kAlpha =
        (kRho * kDartArea * kCd) / (2.0f * kMass);

    // ── Private helpers ─────────────────────────

    /** Numeric integration fallback when no table is loaded. */
    float analytic_drop(float range, float v0) const;

    /** Analytic time-of-flight via closed-form horizontal drag solution. */
    float analytic_time_of_flight(float range, float v0) const;

    /** Bilinear interpolation over the 2-D drop table. */
    float bilinear_interpolate(float range, float v0) const;
};

} // namespace autotrigger
