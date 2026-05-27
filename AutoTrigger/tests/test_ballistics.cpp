#include "autotrigger/ballistics.h"
#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <vector>

// ──────────────────────────────────────────────
// Helper: create a binary table file with given data
// ──────────────────────────────────────────────
namespace {

void write_table_file(const std::string& path,
                      const float* data, std::size_t count) {
    std::ofstream f(path, std::ios::binary);
    ASSERT_TRUE(f.good());
    f.write(reinterpret_cast<const char*>(data),
            static_cast<std::streamsize>(count * sizeof(float)));
    f.close();
}

void write_zero_table(const std::string& path) {
    // Full 801x3 zero-initialised table for deterministic tests
    std::vector<float> zeros(801 * 3, 0.0f);
    write_table_file(path, zeros.data(), zeros.size());
}

} // namespace

// ──────────────────────────────────────────────
// Test 1: Range 0 → zero drop, zero time-of-flight
// ──────────────────────────────────────────────
TEST(BallisticsTest, ZeroRange) {
    autotrigger::Ballistics bal;
    auto result = bal.compute_aimpoint(0.0f, 70.0f);
    EXPECT_NEAR(result.drop_pixels, 0.0f, 0.1f);
    EXPECT_NEAR(result.time_of_flight, 0.0f, 0.001f);
    EXPECT_FALSE(result.out_of_range);
}

// ──────────────────────────────────────────────
// Test 2: 40 m, v0=70 m/s → drop within physically-plausible bounds.
// NOTE: the precomputed table uses a 2 g dart and quadratic drag;
//       actual drop at 40 m is ~3.3 m → ~44 px with default fy=530.
TEST(BallisticsTest, FortyMetersDrop) {
    autotrigger::Ballistics bal;
    auto result = bal.compute_aimpoint(40.0f, 70.0f);
    // Drop must be > 0.5 px (gravity always acts) and < 100 px
    EXPECT_GT(result.drop_pixels, 0.5f);
    EXPECT_LT(result.drop_pixels, 100.0f);
    // Time-of-flight between 0.3 s and 2.0 s at 40 m
    EXPECT_GT(result.time_of_flight, 0.3f);
    EXPECT_LT(result.time_of_flight, 2.0f);
    EXPECT_FALSE(result.out_of_range);
}

// ──────────────────────────────────────────────
// Test 3: Higher velocity → less drop
// ──────────────────────────────────────────────
TEST(BallisticsTest, HigherVelocityLessDrop) {
    autotrigger::Ballistics bal;
    auto r1 = bal.compute_aimpoint(40.0f, 55.0f);  // slow
    auto r2 = bal.compute_aimpoint(40.0f, 90.0f);  // fast
    EXPECT_LT(r2.drop_pixels, r1.drop_pixels);
    EXPECT_LT(r2.time_of_flight, r1.time_of_flight);
}

// ──────────────────────────────────────────────
// Test 4: Drop is monotonic with range
// ──────────────────────────────────────────────
TEST(BallisticsTest, MonotonicDrop) {
    autotrigger::Ballistics bal;
    auto r1 = bal.compute_aimpoint(10.0f, 70.0f);
    auto r2 = bal.compute_aimpoint(30.0f, 70.0f);
    auto r3 = bal.compute_aimpoint(50.0f, 70.0f);
    EXPECT_LT(r1.drop_pixels, r2.drop_pixels);
    EXPECT_LT(r2.drop_pixels, r3.drop_pixels);
}

// ──────────────────────────────────────────────
// Test 5: Determinism — same input → same output
// ──────────────────────────────────────────────
TEST(BallisticsTest, DeterministicLookup) {
    autotrigger::Ballistics bal;
    auto r1 = bal.compute_aimpoint(30.0f, 70.0f);
    auto r2 = bal.compute_aimpoint(30.0f, 70.0f);
    EXPECT_NEAR(r1.drop_pixels, r2.drop_pixels, 0.001f);
    EXPECT_NEAR(r1.time_of_flight, r2.time_of_flight, 0.001f);
}

// ──────────────────────────────────────────────
// Test 6: Out-of-range range → flag set, still returns value
// ──────────────────────────────────────────────
TEST(BallisticsTest, OutOfRangeExtrapolation) {
    autotrigger::Ballistics bal;
    auto result = bal.compute_aimpoint(85.0f, 70.0f);  // beyond 80 m
    EXPECT_TRUE(result.out_of_range);
    EXPECT_GT(result.drop_pixels, 0.0f);  // graceful degradation
}

// ──────────────────────────────────────────────
// Test 7: Invalid velocity → no crash, flag set or zero
// ──────────────────────────────────────────────
TEST(BallisticsTest, InvalidVelocityFallback) {
    autotrigger::Ballistics bal;
    auto r1 = bal.compute_aimpoint(40.0f, 0.0f);    // v₀ = 0
    auto r2 = bal.compute_aimpoint(40.0f, -10.0f);  // negative
    // No crash — just check that results are sane
    EXPECT_TRUE(r1.out_of_range || r1.drop_pixels >= 0.0f);
    EXPECT_TRUE(r2.out_of_range || r2.drop_pixels >= 0.0f);
}

// ──────────────────────────────────────────────
// Test 8: Load table from binary file
// ──────────────────────────────────────────────
TEST(BallisticsTest, LoadTableFromFile) {
    std::ofstream f("test_table.bin", std::ios::binary);
    float data[] = {0.0f, 0.1f, 0.3f, 0.6f};
    f.write(reinterpret_cast<char*>(data), sizeof(data));
    f.close();

    autotrigger::Ballistics bal;
    bool ok = bal.load_table("test_table.bin");
    EXPECT_TRUE(ok);
}

// ──────────────────────────────────────────────
// Test 9: is_loaded() flips after load_table()
// ──────────────────────────────────────────────
TEST(BallisticsTest, IsLoadedFlag) {
    autotrigger::Ballistics bal;
    EXPECT_FALSE(bal.is_loaded());

    std::ofstream f("test_table2.bin", std::ios::binary);
    std::vector<float> zeros(801 * 3, 0.0f);
    f.write(reinterpret_cast<const char*>(zeros.data()),
            static_cast<std::streamsize>(zeros.size() * sizeof(float)));
    f.close();

    bool ok = bal.load_table("test_table2.bin");
    EXPECT_TRUE(ok);
    EXPECT_TRUE(bal.is_loaded());
}

// ──────────────────────────────────────────────
// Test 10: Missing file → load_table() returns false
// ──────────────────────────────────────────────
TEST(BallisticsTest, LoadMissingFile) {
    autotrigger::Ballistics bal;
    bool ok = bal.load_table("nonexistent_file.bin");
    EXPECT_FALSE(ok);
    EXPECT_FALSE(bal.is_loaded());
}

// ──────────────────────────────────────────────
// Test 11: set_fy() changes pixel output proportionally
// ──────────────────────────────────────────────
TEST(BallisticsTest, FocalLengthScaling) {
    autotrigger::Ballistics bal;
    // Use a zero-drop table so we can isolate fy effect.
    // With table all zeros, drop_pixels = 0 regardless of fy.
    // Instead, load a full table and then test.
    // We'll test qualitatively: doubling fy doubles pixels.
    // For now verify the setter/getter pattern works.

    // Create a valid table with a known drop at 40 m, v0=70 m/s
    // index 400 (40m / 0.1m step), velocity column 1 (70 m/s)
    std::vector<float> tbl(801 * 3, 0.0f);
    tbl[400 * 3 + 1] = 0.5f;  // 0.5 m drop at 40 m, 70 m/s
    write_table_file("test_fy.bin", tbl.data(), tbl.size());

    autotrigger::Ballistics bal1;
    bal1.load_table("test_fy.bin");
    auto r1 = bal1.compute_aimpoint(40.0f, 70.0f);

    autotrigger::Ballistics bal2;
    bal2.set_fy(1060.0f);  // double the default
    bal2.load_table("test_fy.bin");
    auto r2 = bal2.compute_aimpoint(40.0f, 70.0f);

    // Double fy → double drop_pixels (drop_meters same)
    EXPECT_NEAR(r2.drop_pixels, r1.drop_pixels * 2.0f, 0.01f);
}

// ─────────────────────────────────────────────────────
// Helper: build a physically-plausible 801×3 drop table
// ─────────────────────────────────────────────────────
namespace {

void write_drop_table(const std::string& path) {
    std::vector<float> tbl(801 * 3, 0.0f);
    for (int i = 0; i < 801; ++i) {
        float r = static_cast<float>(i) * 0.1f;
        for (int j = 0; j < 3; ++j) {
            float v = (j == 0) ? 55.0f : ((j == 1) ? 70.0f : 90.0f);
            // Approximate drop: g·r²/(2·v²) × 1.4 drag multiplier
            tbl[i * 3 + j] = 9.81f * r * r / (2.0f * v * v) * 1.4f;
        }
    }
    write_table_file(path, tbl.data(), tbl.size());
}

} // namespace

// ══════════════════════════════════════════════════════
//  Configurable ballistics physics — RED phase
//
//  These tests exercise set_config() / config() which
//  are DECLARED in ballistics.h but NOT YET IMPLEMENTED.
//  Tests MUST COMPILE (link failure is expected).
// ══════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────
// Test 12: Doubling dart mass → less drop
//
//   alpha = (ρ·A·Cd) / (2·m)   →   α ∝ 1/m
//   double mass → half alpha → half deceleration → less drop
// ─────────────────────────────────────────────────────
TEST(BallisticsTest, DoubleMassLessDrop) {
    write_drop_table("test_cfg_doublemass.bin");

    autotrigger::Ballistics bal;
    ASSERT_TRUE(bal.load_table("test_cfg_doublemass.bin"));

    // Baseline: default config (dart_mass_kg = 0.002)
    auto ref = bal.compute_aimpoint(30.0f, 70.0f);
    ASSERT_GT(ref.drop_pixels, 0.0f);

    // Double the dart mass → expect LESS drop
    autotrigger::BallisticsConfig cfg;
    cfg.dart_mass_kg = 0.004f;  // 2× default
    bal.set_config(cfg);
    auto result = bal.compute_aimpoint(30.0f, 70.0f);

    EXPECT_LT(result.drop_pixels, ref.drop_pixels);
}

// ─────────────────────────────────────────────────────
// Test 13: Doubling drag coefficient → more drop
//
//   alpha ∝ Cd  →  double Cd → double alpha → more drop
// ─────────────────────────────────────────────────────
TEST(BallisticsTest, DoubleCdMoreDrop) {
    write_drop_table("test_cfg_doublecd.bin");

    autotrigger::Ballistics bal;
    ASSERT_TRUE(bal.load_table("test_cfg_doublecd.bin"));

    // Baseline: default config (drag_coefficient = 0.67)
    auto ref = bal.compute_aimpoint(30.0f, 70.0f);
    ASSERT_GT(ref.drop_pixels, 0.0f);

    // Double Cd → expect MORE drop
    autotrigger::BallisticsConfig cfg;
    cfg.drag_coefficient = 1.34f;  // 2× default
    bal.set_config(cfg);
    auto result = bal.compute_aimpoint(30.0f, 70.0f);

    EXPECT_GT(result.drop_pixels, ref.drop_pixels);
}

// ─────────────────────────────────────────────────────
// Test 14: Identical custom config → bit-identical output
// ─────────────────────────────────────────────────────
TEST(BallisticsTest, CustomConfigDeterministic) {
    write_drop_table("test_cfg_deterministic.bin");

    autotrigger::BallisticsConfig cfg;
    cfg.dart_mass_kg     = 0.003f;
    cfg.drag_coefficient = 0.8f;

    autotrigger::Ballistics bal1;
    ASSERT_TRUE(bal1.load_table("test_cfg_deterministic.bin"));
    bal1.set_config(cfg);
    auto r1 = bal1.compute_aimpoint(25.0f, 55.0f);

    autotrigger::Ballistics bal2;
    ASSERT_TRUE(bal2.load_table("test_cfg_deterministic.bin"));
    bal2.set_config(cfg);
    auto r2 = bal2.compute_aimpoint(25.0f, 55.0f);

    // Same config + same table → MUST be bit-identical
    EXPECT_FLOAT_EQ(r1.drop_pixels, r2.drop_pixels);
    EXPECT_FLOAT_EQ(r1.time_of_flight, r2.time_of_flight);
}

// ─────────────────────────────────────────────────────
// Test 15: Explicit default config matches implicit default
// ─────────────────────────────────────────────────────
TEST(BallisticsTest, DefaultConfigMatchesLegacy) {
    write_drop_table("test_cfg_legacy.bin");

    // Instance A: no set_config() call (implicit defaults)
    autotrigger::Ballistics bal_no_config;
    ASSERT_TRUE(bal_no_config.load_table("test_cfg_legacy.bin"));
    auto r1 = bal_no_config.compute_aimpoint(40.0f, 70.0f);

    // Instance B: explicit set_config(BallisticsConfig{})
    autotrigger::Ballistics bal_explicit;
    ASSERT_TRUE(bal_explicit.load_table("test_cfg_legacy.bin"));
    autotrigger::BallisticsConfig cfg;  // all-default values
    bal_explicit.set_config(cfg);
    auto r2 = bal_explicit.compute_aimpoint(40.0f, 70.0f);

    EXPECT_FLOAT_EQ(r1.drop_pixels, r2.drop_pixels);
    EXPECT_FLOAT_EQ(r1.time_of_flight, r2.time_of_flight);
}

// ─────────────────────────────────────────────────────
// Test 16: set_config() / config() round-trip integrity
// ─────────────────────────────────────────────────────
TEST(BallisticsTest, SetConfigGetConfigRoundTrip) {
    autotrigger::Ballistics bal;

    autotrigger::BallisticsConfig c1;
    c1.dart_mass_kg     = 0.003f;
    c1.drag_coefficient = 0.8f;
    c1.dart_area_m2     = 1.5e-4f;
    c1.air_density      = 1.1f;
    c1.gravity          = 9.78f;

    bal.set_config(c1);
    const autotrigger::BallisticsConfig& c2 = bal.config();

    EXPECT_FLOAT_EQ(c1.dart_mass_kg,     c2.dart_mass_kg);
    EXPECT_FLOAT_EQ(c1.drag_coefficient, c2.drag_coefficient);
    EXPECT_FLOAT_EQ(c1.dart_area_m2,     c2.dart_area_m2);
    EXPECT_FLOAT_EQ(c1.air_density,      c2.air_density);
    EXPECT_FLOAT_EQ(c1.gravity,          c2.gravity);
}
