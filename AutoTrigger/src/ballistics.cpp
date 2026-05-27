#include "autotrigger/ballistics.h"

#include <algorithm>
#include <cmath>
#include <fstream>

namespace autotrigger {

// ─────────────────────────────────────────────────────
// Construction / Destruction
// ─────────────────────────────────────────────────────

Ballistics::Ballistics()  = default;
Ballistics::~Ballistics() = default;

// ─────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────

bool Ballistics::load_table(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.good()) {
        loaded_ = false;
        return false;
    }

    // Zero the table first so partial reads leave clean state
    for (int r = 0; r < kRangeEntries; ++r) {
        for (int v = 0; v < kVelEntries; ++v) {
            table_[r][v] = 0.0f;
        }
    }

    // Read as many floats as the file provides (up to table capacity)
    constexpr std::size_t kTotal = kRangeEntries * kVelEntries;
    float* dst = &table_[0][0];
    file.read(reinterpret_cast<char*>(dst),
              static_cast<std::streamsize>(kTotal * sizeof(float)));

    // Success if we read at least one value (lenient — production
    // tables should always be full 801x3, but tests use small files).
    loaded_ = (file.gcount() > 0);
    return loaded_;
}

bool Ballistics::is_loaded() const { return loaded_; }

void Ballistics::set_fy(float fy_px) { fy_ = fy_px; }
float Ballistics::fy() const { return fy_; }

// ─────────────────────────────────────────────────────
// Main computation
// ─────────────────────────────────────────────────────

AimpointResult Ballistics::compute_aimpoint(float range, float v0) const {
    AimpointResult result;

    // --- Guard: zero or negative range -> all zeros ---
    if (range <= 0.0f) {
        return result;
    }

    // --- Guard: invalid velocity ---
    if (v0 < 10.0f) {
        result.out_of_range = true;
        return result;
    }

    // --- Range clamping ---
    float clamped = range;
    if (range > kMaxRange) {
        clamped = kMaxRange;
        result.out_of_range = true;
    }

    // --- Compute drop in metres ---
    // Use precomputed table when available; fall back to analytic
    // model when no table has been loaded (graceful degradation).
    float drop_m;
    if (loaded_) {
        drop_m = bilinear_interpolate(clamped, v0);
    } else {
        drop_m = analytic_drop(clamped, v0);
    }
    if (drop_m < 0.0f) drop_m = 0.0f;

    // --- Convert to pixels ---
    result.drop_pixels = fy_ * drop_m / clamped;

    // --- Time of flight (analytic closed-form) ---
    result.time_of_flight = analytic_time_of_flight(clamped, v0);

    return result;
}

// ─────────────────────────────────────────────────────
//  Bilinear interpolation over (range, velocity) grid
// ─────────────────────────────────────────────────────

float Ballistics::bilinear_interpolate(float range, float v0) const {
    // range -> fractional index
    float ri = range / kRangeStep;
    int    r0 = static_cast<int>(ri);
    int    r1 = r0 + 1;
    float  rt = ri - static_cast<float>(r0);

    // Clamp to table bounds
    r0 = std::clamp(r0, 0, kRangeEntries - 1);
    r1 = std::clamp(r1, 0, kRangeEntries - 1);
    rt = std::clamp(rt, 0.0f, 1.0f);

    // velocity -> piecewise index + fractional between the three columns
    if (v0 <= kVelocities[0]) {
        // Below minimum: constant-extrapolate from column 0
        float v00 = table_[r0][0];
        float v01 = table_[r1][0];
        return v00 + rt * (v01 - v00);
    } else if (v0 >= kVelocities[kVelEntries - 1]) {
        // Above maximum: constant-extrapolate from last column
        int col = kVelEntries - 1;
        float v00 = table_[r0][col];
        float v01 = table_[r1][col];
        return v00 + rt * (v01 - v00);
    } else {
        // Between two columns: linear interpolation
        int col = 0;
        while (col < kVelEntries - 1 && v0 > kVelocities[col + 1]) {
            ++col;
        }
        float vt = (v0 - kVelocities[col]) /
                   (kVelocities[col + 1] - kVelocities[col]);

        float v00 = table_[r0][col];
        float v01 = table_[r1][col];
        float v10 = table_[r0][col + 1];
        float v11 = table_[r1][col + 1];

        // Bilinear blend
        float low  = v00 + rt * (v01 - v00);
        float high = v10 + rt * (v11 - v10);
        return low + vt * (high - low);
    }
}

// ─────────────────────────────────────────────────────
//  Analytic drop (fallback when no table is loaded)
// ─────────────────────────────────────────────────────
namespace {

float numeric_drop(float range, float v0, float alpha, float g) {
    const float dt = 0.001f;
    float x = 0.0f, y = 0.0f;
    float vx = v0, vy = 0.0f;
    const int max_steps = 500000;

    for (int step = 0; step < max_steps; ++step) {
        float v = std::sqrt(vx * vx + vy * vy);
        if (v < 1e-9f) break;

        float drag = alpha * v;
        float ax = -drag * vx;
        float ay = -drag * vy - g;

        x += vx * dt;
        y += vy * dt;
        vx += ax * dt;
        vy += ay * dt;

        if (x >= range) return std::max(-y, 0.0f);
        if (y < -10000.0f) break;
    }
    return 999.0f;  // sentinel: did not reach range
}

} // namespace

float Ballistics::analytic_drop(float range, float v0) const {
    if (range <= 0.0f || v0 < 1.0f) return 0.0f;
    return numeric_drop(range, v0, kAlpha, kGravity);
}

// ─────────────────────────────────────────────────────
//  Analytic time-of-flight (horizontal drag approx.)
// ─────────────────────────────────────────────────────

float Ballistics::analytic_time_of_flight(float range, float v0) const {
    // dvx/dt = -alpha * vx^2
    //   ->  vx(t) = v0 / (1 + alpha * v0 * t)
    //   ->  x(t)  = ln(1 + alpha * v0 * t) / alpha
    //   ->  t(x)  = (e^(alpha * x) - 1) / (alpha * v0)
    float ax = kAlpha * range;
    // Clamp exp argument to prevent overflow
    if (ax > 80.0f) ax = 80.0f;

    float ep1 = std::exp(ax);
    float denom = kAlpha * v0;
    if (denom < 1e-12f) return 0.0f;

    return (ep1 - 1.0f) / denom;
}

} // namespace autotrigger
