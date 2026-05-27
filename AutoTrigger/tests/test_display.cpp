#include "autotrigger/display.h"
#include <gtest/gtest.h>

using autotrigger::DisplayMock;

// Test 1: Crosshair renders at center
TEST(DisplayTest, CrosshairAtCenter) {
    DisplayMock d;
    d.init();
    d.render(0, 0);
    // Vertical crosshair line at x=119,120 (2px wide)
    EXPECT_GT(d.get_pixel(119, 119), 0u);
    EXPECT_GT(d.get_pixel(119, 120), 0u);
    // Horizontal crosshair line at y=119,120
    EXPECT_GT(d.get_pixel(118, 119), 0u);
    EXPECT_GT(d.get_pixel(120, 119), 0u);
}

// Test 2: Aim-point circle drawn at given coords
TEST(DisplayTest, AimPointDrawnAtGivenCoords) {
    DisplayMock d;
    d.init();
    d.render(100, 80);
    // Center of circle should be bright
    EXPECT_GT(d.get_pixel(100, 80), 0u);
    // 5px away from center (outside radius 5) should be 0
    EXPECT_EQ(d.get_pixel(100, 70), 0u);
    EXPECT_EQ(d.get_pixel(90, 80), 0u);
}

// Test 3: Aim-point out-of-bounds clamped, no crash
TEST(DisplayTest, AimPointOutOfBoundsClampedNoCrash) {
    DisplayMock d;
    d.init();
    // Verify render does not crash with extreme values
    d.render(-10, 0);
    d.render(300, -5);
    d.render(-5, 300);
    d.render(300, 300);
    // Aim-point clamped inside 240x240 — round mask may black corners,
    // but at least the crosshair center (120,120) is always visible.
    EXPECT_GT(d.get_pixel(120, 120), 0u);
}

// Test 4: Round mask - corners are black
TEST(DisplayTest, RoundMaskCornersBlack) {
    DisplayMock d;
    d.init();
    d.render(0, 0);
    // Corner (0,0): distance from center = sqrt(120^2+120^2) = 169.7 > 115
    EXPECT_EQ(d.get_pixel(0, 0), 0u);
    EXPECT_EQ(d.get_pixel(239, 0), 0u);
    EXPECT_EQ(d.get_pixel(0, 239), 0u);
    EXPECT_EQ(d.get_pixel(239, 239), 0u);
}

// Test 5: Center is visible (inside circle)
TEST(DisplayTest, CenterVisible) {
    DisplayMock d;
    d.init();
    d.render(0, 0);
    // Center should have crosshair pixels lit
    EXPECT_GT(d.get_pixel(120, 120), 0u);
}

// Test 6: Fresh render clears previous aim-point
TEST(DisplayTest, FreshRenderClearsPreviousAimPoint) {
    DisplayMock d;
    d.init();
    d.render(100, 80);
    EXPECT_GT(d.get_pixel(100, 80), 0u);
    d.render(200, 80);
    // Old position should now be 0
    EXPECT_EQ(d.get_pixel(100, 80), 0u);
    // New position should be bright
    EXPECT_GT(d.get_pixel(200, 80), 0u);
}
