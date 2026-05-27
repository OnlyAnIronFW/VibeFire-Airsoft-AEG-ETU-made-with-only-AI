#include "autotrigger/safety.h"

// HAL interface needed for MockTrigger definition
#include "autotrigger/hal/itrigger.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>

using testing::AtLeast;
using testing::StrictMock;

// ────────────────────────────────────────────────────────────
// GoogleMock for ITrigger — needed to verify trigger stays LOW
// ────────────────────────────────────────────────────────────
namespace {

using namespace autotrigger;

class MockTrigger : public ITrigger {
public:
    MOCK_METHOD(bool, init, (), (override));
    MOCK_METHOD(bool, is_held, (), (override));
    MOCK_METHOD(void, fire, (bool), (override));
    MOCK_METHOD(void, release, (), (override));
};

// ────────────────────────────────────────────────────────────
// Startup tests
// ────────────────────────────────────────────────────────────

TEST(SafetyStartupTest, AllOk_Proceeds) {
    StrictMock<MockTrigger> trigger;
    // trigger->fire(false) is called during startup
    EXPECT_CALL(trigger, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &trigger);

    auto result = safety.do_startup_check();

    EXPECT_TRUE(result.can_proceed);
    EXPECT_FALSE(result.degraded);
    EXPECT_TRUE(safety.is_startup_ok());
    EXPECT_FALSE(safety.is_startup_degraded());
    EXPECT_TRUE(safety.is_safe());
}

TEST(SafetyStartupTest, CameraAbsent_ReturnsFalse) {
    StrictMock<MockTrigger> trigger;
    EXPECT_CALL(trigger, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &trigger);
    safety.set_mock_camera_present(false);

    auto result = safety.do_startup_check();

    EXPECT_FALSE(result.can_proceed);
    EXPECT_FALSE(safety.is_startup_ok());
}

TEST(SafetyStartupTest, ToFTimeout_DegradedMode) {
    StrictMock<MockTrigger> trigger;
    EXPECT_CALL(trigger, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &trigger);
    safety.set_mock_tof_healthy(false);

    auto result = safety.do_startup_check();

    EXPECT_TRUE(result.can_proceed);
    EXPECT_TRUE(result.degraded);
    EXPECT_TRUE(safety.is_startup_ok());
    EXPECT_TRUE(safety.is_startup_degraded());
    // Trigger stays LOW — is_safe() is false in degraded mode
    EXPECT_FALSE(safety.is_safe());
}

TEST(SafetyStartupTest, ToFTimeout_RecoverViaRunOnce) {
    StrictMock<MockTrigger> trigger;
    EXPECT_CALL(trigger, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &trigger);
    safety.set_mock_tof_healthy(false);

    auto result = safety.do_startup_check();
    EXPECT_TRUE(result.degraded);
    EXPECT_FALSE(safety.is_safe());

    // Later, ToF becomes healthy again
    safety.set_mock_tof_healthy(true);
    safety.run_once();

    EXPECT_FALSE(safety.is_startup_degraded());
    EXPECT_TRUE(safety.is_safe());
}

TEST(SafetyStartupTest, GpioDefaultsFail_ReturnsFalse) {
    StrictMock<MockTrigger> trigger;
    EXPECT_CALL(trigger, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &trigger);
    safety.set_mock_gpio_defaults_ok(false);

    auto result = safety.do_startup_check();

    EXPECT_FALSE(result.can_proceed);
    EXPECT_FALSE(safety.is_startup_ok());
}

// ────────────────────────────────────────────────────────────
// Thermal monitor tests
// ────────────────────────────────────────────────────────────

TEST(SafetyThermalTest, NormalTemp_NotThrottled) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);
    safety.set_mock_temperature(50.0f);

    safety.monitor_thermal();

    EXPECT_FALSE(safety.is_throttled());
    EXPECT_NEAR(safety.current_temperature(), 50.0f, 0.1f);
}

TEST(SafetyThermalTest, AboveThreshold_Throttles) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);
    safety.set_mock_temperature(90.0f);

    safety.monitor_thermal();

    EXPECT_TRUE(safety.is_throttled());
}

TEST(SafetyThermalTest, ExactlyAtThreshold_Throttles) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);
    safety.set_mock_temperature(Safety::kThrottleOnTemp);

    safety.monitor_thermal();

    EXPECT_TRUE(safety.is_throttled());
}

TEST(SafetyThermalTest, Hysteresis_ClearsBelowOffTemp) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);

    // Heat up
    safety.set_mock_temperature(90.0f);
    safety.monitor_thermal();
    EXPECT_TRUE(safety.is_throttled());

    // Cool down — still above kThrottleOffTemp
    safety.set_mock_temperature(83.0f);
    safety.monitor_thermal();
    EXPECT_TRUE(safety.is_throttled());

    // Cool further — below kThrottleOffTemp
    safety.set_mock_temperature(75.0f);
    safety.monitor_thermal();
    EXPECT_FALSE(safety.is_throttled());
}

// ────────────────────────────────────────────────────────────
// Watchdog tests
// ────────────────────────────────────────────────────────────

TEST(SafetyWatchdogTest, KickThenCheck_Alive) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);

    safety.kick_watchdog();

    EXPECT_TRUE(safety.is_watchdog_alive());
    EXPECT_EQ(safety.watchdog_kick_count(), 1);
}

TEST(SafetyWatchdogTest, NotKicked_ExpiresAfter100ms) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);

    safety.kick_watchdog();
    EXPECT_TRUE(safety.is_watchdog_alive());

    // Advance past the 100 ms timeout
    safety.advance_time_ms(101);

    EXPECT_FALSE(safety.is_watchdog_alive());
}

TEST(SafetyWatchdogTest, KickWithinWindow_StaysAlive) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);

    safety.kick_watchdog();
    safety.advance_time_ms(50);  // Still within 100 ms
    EXPECT_TRUE(safety.is_watchdog_alive());

    safety.kick_watchdog();      // Refresh
    safety.advance_time_ms(50);
    EXPECT_TRUE(safety.is_watchdog_alive());

    safety.advance_time_ms(60);  // 110 ms since last kick
    EXPECT_FALSE(safety.is_watchdog_alive());
}

TEST(SafetyWatchdogTest, MsSinceLastKick) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);

    safety.kick_watchdog();
    EXPECT_EQ(safety.ms_since_last_watchdog_kick().count(), 0);

    safety.advance_time_ms(33);
    EXPECT_EQ(safety.ms_since_last_watchdog_kick().count(), 33);
}

// ────────────────────────────────────────────────────────────
// Heartbeat tests
// ────────────────────────────────────────────────────────────

TEST(SafetyHeartbeatTest, TogglesAt1Hz) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);

    // First call → toggle ON
    safety.heartbeat();
    EXPECT_TRUE(safety.heartbeat_state());
    EXPECT_EQ(safety.heartbeat_toggle_count(), 1);
    EXPECT_TRUE(safety.last_heartbeat_setpoint());

    // Advance 500 ms → toggle OFF
    safety.advance_time_ms(500);
    safety.heartbeat();
    EXPECT_FALSE(safety.heartbeat_state());
    EXPECT_EQ(safety.heartbeat_toggle_count(), 2);
    EXPECT_FALSE(safety.last_heartbeat_setpoint());

    // Advance 500 ms → toggle ON
    safety.advance_time_ms(500);
    safety.heartbeat();
    EXPECT_TRUE(safety.heartbeat_state());
    EXPECT_EQ(safety.heartbeat_toggle_count(), 3);
}

TEST(SafetyHeartbeatTest, DoesNotToggleBeforePeriod) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);

    safety.heartbeat();  // toggles ON
    EXPECT_TRUE(safety.heartbeat_state());

    // Call again quickly — should NOT toggle
    safety.advance_time_ms(200);
    safety.heartbeat();
    EXPECT_TRUE(safety.heartbeat_state());  // still ON
    EXPECT_EQ(safety.heartbeat_toggle_count(), 1);
}

TEST(SafetyHeartbeatTest, TogglesAtBoundary) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);

    safety.heartbeat();  // ON

    safety.advance_time_ms(499);  // 1 ms before period
    safety.heartbeat();
    EXPECT_TRUE(safety.heartbeat_state());   // not yet
    EXPECT_EQ(safety.heartbeat_toggle_count(), 1);

    safety.advance_time_ms(2);    // 501 ms total from first toggle
    safety.heartbeat();
    EXPECT_FALSE(safety.heartbeat_state());  // toggled OFF
    EXPECT_EQ(safety.heartbeat_toggle_count(), 2);
}

// ────────────────────────────────────────────────────────────
// Integration: is_safe() combines all checks
// ────────────────────────────────────────────────────────────

TEST(SafetyIntegrationTest, IsSafe_OnlyTrueWhenAllOk) {
    StrictMock<MockTrigger> trigger;
    EXPECT_CALL(trigger, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &trigger);

    // Before startup: not safe
    EXPECT_FALSE(safety.is_safe());

    // After successful startup: safe
    safety.do_startup_check();
    EXPECT_TRUE(safety.is_safe());

    // Thermal throttle → not safe
    safety.set_mock_temperature(90.0f);
    safety.monitor_thermal();
    EXPECT_FALSE(safety.is_safe());

    // Cool down → safe again
    safety.set_mock_temperature(70.0f);
    safety.monitor_thermal();
    EXPECT_TRUE(safety.is_safe());

    // Watchdog expires → not safe
    safety.advance_time_ms(150);
    EXPECT_FALSE(safety.is_safe());

    // Kick watchdog → safe again
    safety.kick_watchdog();
    EXPECT_TRUE(safety.is_safe());
}

TEST(SafetyIntegrationTest, RunOnce_DoesNotAutoKickWatchdog) {
    StrictMock<MockTrigger> trigger;
    SafetyMock safety(nullptr, nullptr, &trigger);

    safety.kick_watchdog();
    safety.advance_time_ms(50);
    safety.run_once();  // Does NOT kick watchdog
    EXPECT_TRUE(safety.is_watchdog_alive());

    safety.advance_time_ms(60);  // Total 110 ms since last kick
    EXPECT_FALSE(safety.is_watchdog_alive());
}

} // namespace
