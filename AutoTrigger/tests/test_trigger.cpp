#include "autotrigger/trigger.h"
#include <gtest/gtest.h>

// ============================================================
// Trigger tests — 3-frame aim consensus via TriggerMock
// ============================================================
// The TriggerMock provides injectable is_held() and observable
// fire() calls so the consensus logic can be fully verified
// without real GPIO hardware.
// ============================================================

// ──────────────────────────────────────────────
// Test 1: Held + aim aligned × 3 → fire HIGH
// ──────────────────────────────────────────────
TEST(TriggerTest, ThreeConsecutiveAlignedFramesFires) {
    autotrigger::TriggerMock mock;
    mock.inject_held_ = true;

    // Frame 1 — consensus_counter = 1
    mock.update(true);
    EXPECT_FALSE(mock.last_fire_);
    EXPECT_EQ(mock.fire_call_count_, 1);

    // Frame 2 — consensus_counter = 2
    mock.update(true);
    EXPECT_FALSE(mock.last_fire_);
    EXPECT_EQ(mock.fire_call_count_, 2);

    // Frame 3 — consensus_counter = 3 → fire!
    mock.update(true);
    EXPECT_TRUE(mock.last_fire_);
    EXPECT_EQ(mock.fire_call_count_, 3);
}

// ──────────────────────────────────────────────
// Test 2: Held + aim NOT aligned → fire LOW
// ──────────────────────────────────────────────
TEST(TriggerTest, HeldButAimNotAlignedStaysLow) {
    autotrigger::TriggerMock mock;
    mock.inject_held_ = true;

    mock.update(false);
    EXPECT_FALSE(mock.last_fire_);
}

// ──────────────────────────────────────────────
// Test 3: Aim aligned but trigger NOT held → fire LOW
// ──────────────────────────────────────────────
TEST(TriggerTest, AimAlignedButTriggerNotHeldStaysLow) {
    autotrigger::TriggerMock mock;
    mock.inject_held_ = false;

    // Even many frames of alignment should not fire
    for (int i = 0; i < 5; ++i) {
        mock.update(true);
        EXPECT_FALSE(mock.last_fire_);
    }
}

// ──────────────────────────────────────────────
// Test 4: 2 frames only → no partial consensus
// ──────────────────────────────────────────────
TEST(TriggerTest, TwoFramesNotEnoughForConsensus) {
    autotrigger::TriggerMock mock;
    mock.inject_held_ = true;

    mock.update(true);
    EXPECT_FALSE(mock.last_fire_);

    mock.update(true);
    EXPECT_FALSE(mock.last_fire_);
}

// ──────────────────────────────────────────────
// Test 5: Aim lost mid-fire → drops LOW immediately
// ──────────────────────────────────────────────
TEST(TriggerTest, AimLostMidFireDropsImmediately) {
    autotrigger::TriggerMock mock;
    mock.inject_held_ = true;

    // Build consensus and fire
    mock.update(true);
    mock.update(true);
    mock.update(true);
    ASSERT_TRUE(mock.last_fire_);

    // Lose aim — must drop immediately
    mock.update(false);
    EXPECT_FALSE(mock.last_fire_);
}

// ──────────────────────────────────────────────
// Test 6: Startup → output LOW by default
// ──────────────────────────────────────────────
TEST(TriggerTest, StartupOutputIsLow) {
    autotrigger::TriggerMock mock;
    EXPECT_FALSE(mock.last_fire_);
    EXPECT_EQ(mock.fire_call_count_, 0);

    // Even after an update with no alignment, output stays low
    mock.update(false);
    EXPECT_FALSE(mock.last_fire_);
}

// ──────────────────────────────────────────────
// Test 7: Re-acquired aim → re-fires after 3 frames
// ──────────────────────────────────────────────
TEST(TriggerTest, AimReacquiredRefiresAfterThreeFrames) {
    autotrigger::TriggerMock mock;
    mock.inject_held_ = true;

    // Phase 1: acquire and fire
    mock.update(true);
    mock.update(true);
    mock.update(true);
    ASSERT_TRUE(mock.last_fire_);

    // Phase 2: lose aim → stop
    mock.update(false);
    ASSERT_FALSE(mock.last_fire_);

    // Phase 3: re-acquire — need 3 new consecutive frames
    mock.update(true);
    EXPECT_FALSE(mock.last_fire_);

    mock.update(true);
    EXPECT_FALSE(mock.last_fire_);

    mock.update(true);
    EXPECT_TRUE(mock.last_fire_);
}
