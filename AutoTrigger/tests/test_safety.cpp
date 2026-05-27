#include "autotrigger/safety.h"
#include "autotrigger/ranging.h"

// HAL interface needed for MockFireOutput definition
#include "autotrigger/hal/ifire_output.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>

using testing::AtLeast;
using testing::StrictMock;

// ────────────────────────────────────────────────────────────
// GoogleMock for IFireOutput — needed to verify fire stays LOW
// ────────────────────────────────────────────────────────────
namespace {

using namespace autotrigger;

class MockFireOutput : public IFireOutput {
public:
    MOCK_METHOD(bool, init, (), (override));
    MOCK_METHOD(void, fire, (bool), (override));
    MOCK_METHOD(void, release, (), (override));
};

// ────────────────────────────────────────────────────────────
// Startup tests
// ────────────────────────────────────────────────────────────

TEST(SafetyStartupTest, AllOk_Proceeds) {
    StrictMock<MockFireOutput> fire_output;
    // fire_output->fire(false) is called during startup
    EXPECT_CALL(fire_output, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &fire_output);

    auto result = safety.do_startup_check();

    EXPECT_TRUE(result.can_proceed);
    EXPECT_FALSE(result.degraded);
    EXPECT_TRUE(safety.is_startup_ok());
    EXPECT_FALSE(safety.is_startup_degraded());
    EXPECT_TRUE(safety.is_safe());
}

TEST(SafetyStartupTest, CameraAbsent_ReturnsFalse) {
    StrictMock<MockFireOutput> fire_output;
    EXPECT_CALL(fire_output, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &fire_output);
    safety.set_mock_camera_present(false);

    auto result = safety.do_startup_check();

    EXPECT_FALSE(result.can_proceed);
    EXPECT_FALSE(safety.is_startup_ok());
}

TEST(SafetyStartupTest, ToFTimeout_DegradedMode) {
    StrictMock<MockFireOutput> fire_output;
    EXPECT_CALL(fire_output, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &fire_output);
    safety.set_mock_tof_healthy(false);

    auto result = safety.do_startup_check();

    EXPECT_TRUE(result.can_proceed);
    EXPECT_TRUE(result.degraded);
    EXPECT_TRUE(safety.is_startup_ok());
    EXPECT_TRUE(safety.is_startup_degraded());
    // Fire output stays LOW — is_safe() is false in degraded mode
    EXPECT_FALSE(safety.is_safe());
}

TEST(SafetyStartupTest, ToFTimeout_RecoverViaRunOnce) {
    StrictMock<MockFireOutput> fire_output;
    EXPECT_CALL(fire_output, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &fire_output);
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
    StrictMock<MockFireOutput> fire_output;
    EXPECT_CALL(fire_output, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &fire_output);
    safety.set_mock_gpio_defaults_ok(false);

    auto result = safety.do_startup_check();

    EXPECT_FALSE(result.can_proceed);
    EXPECT_FALSE(safety.is_startup_ok());
}

// ────────────────────────────────────────────────────────────
// Thermal monitor tests
// ────────────────────────────────────────────────────────────

TEST(SafetyThermalTest, NormalTemp_NotThrottled) {
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);
    safety.set_mock_temperature(50.0f);

    safety.monitor_thermal();

    EXPECT_FALSE(safety.is_throttled());
    EXPECT_NEAR(safety.current_temperature(), 50.0f, 0.1f);
}

TEST(SafetyThermalTest, AboveThreshold_Throttles) {
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);
    safety.set_mock_temperature(90.0f);

    safety.monitor_thermal();

    EXPECT_TRUE(safety.is_throttled());
}

TEST(SafetyThermalTest, ExactlyAtThreshold_Throttles) {
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);
    safety.set_mock_temperature(Safety::kThrottleOnTemp);

    safety.monitor_thermal();

    EXPECT_TRUE(safety.is_throttled());
}

TEST(SafetyThermalTest, Hysteresis_ClearsBelowOffTemp) {
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);

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
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);

    safety.kick_watchdog();

    EXPECT_TRUE(safety.is_watchdog_alive());
    EXPECT_EQ(safety.watchdog_kick_count(), 1);
}

TEST(SafetyWatchdogTest, NotKicked_ExpiresAfter100ms) {
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);

    safety.kick_watchdog();
    EXPECT_TRUE(safety.is_watchdog_alive());

    // Advance past the 100 ms timeout
    safety.advance_time_ms(101);

    EXPECT_FALSE(safety.is_watchdog_alive());
}

TEST(SafetyWatchdogTest, KickWithinWindow_StaysAlive) {
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);

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
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);

    safety.kick_watchdog();
    EXPECT_EQ(safety.ms_since_last_watchdog_kick().count(), 0);

    safety.advance_time_ms(33);
    EXPECT_EQ(safety.ms_since_last_watchdog_kick().count(), 33);
}

// ────────────────────────────────────────────────────────────
// Heartbeat tests
// ────────────────────────────────────────────────────────────

TEST(SafetyHeartbeatTest, TogglesAt1Hz) {
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);

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
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);

    safety.heartbeat();  // toggles ON
    EXPECT_TRUE(safety.heartbeat_state());

    // Call again quickly — should NOT toggle
    safety.advance_time_ms(200);
    safety.heartbeat();
    EXPECT_TRUE(safety.heartbeat_state());  // still ON
    EXPECT_EQ(safety.heartbeat_toggle_count(), 1);
}

TEST(SafetyHeartbeatTest, TogglesAtBoundary) {
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);

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
    StrictMock<MockFireOutput> fire_output;
    EXPECT_CALL(fire_output, fire(false)).Times(testing::AtLeast(1));

    SafetyMock safety(nullptr, nullptr, &fire_output);

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
    StrictMock<MockFireOutput> fire_output;
    SafetyMock safety(nullptr, nullptr, &fire_output);

    safety.kick_watchdog();
    safety.advance_time_ms(50);
    safety.run_once();  // Does NOT kick watchdog
    EXPECT_TRUE(safety.is_watchdog_alive());

    safety.advance_time_ms(60);  // Total 110 ms since last kick
    EXPECT_FALSE(safety.is_watchdog_alive());
}

// ────────────────────────────────────────────────────────────
// TDD: ToF sensor check delegation via IRanging::try_ping()
//
// These tests use Safety (NOT SafetyMock) with RangingMock
// to verify that ping_tof_sensor_impl() delegates to
// ranging_->try_ping().  Currently ping_tof_sensor_impl()
// uses raw I²C VL53L1X code — the RangingMock is ignored.
// Tests MUST fail (RED phase) until the implementation is
// refactored to call ranging_->try_ping().
// ────────────────────────────────────────────────────────────

// Test 1: RangingMock reports ping success → startup proceeds cleanly.
//          FAILS because Safety ignores the mock and uses I²C directly.
TEST(SafetyToFDelegationTest, TryPingSuccessProceeds) {
    StrictMock<MockFireOutput> fire_output;
    EXPECT_CALL(fire_output, fire(false)).Times(AtLeast(1));

    RangingMock ranging;
    ranging.set_mock_ping_result(true);   // ToF is alive

    Safety safety(nullptr, &ranging, &fire_output);

    auto result = safety.do_startup_check();

    // When delegation works: can_proceed=true, !degraded, is_safe()=true
    EXPECT_TRUE(result.can_proceed);
    EXPECT_FALSE(result.degraded);
    EXPECT_TRUE(safety.is_startup_ok());
    EXPECT_FALSE(safety.is_startup_degraded());

    // Watchdog starts expired with real Safety; kick it for a fair is_safe() check.
    safety.kick_watchdog();
    EXPECT_TRUE(safety.is_safe());
}

// Test 2: RangingMock reports ping failure → degraded mode.
//          FAILS because Safety ignores the mock and uses I²C directly.
TEST(SafetyToFDelegationTest, TryPingFailureDegrades) {
    StrictMock<MockFireOutput> fire_output;
    EXPECT_CALL(fire_output, fire(false)).Times(AtLeast(1));

    RangingMock ranging;
    ranging.set_mock_ping_result(false);  // ToF is dead

    Safety safety(nullptr, &ranging, &fire_output);

    auto result = safety.do_startup_check();

    // When delegation works: can_proceed=true, degraded=true, is_safe()=false
    EXPECT_TRUE(result.can_proceed);
    EXPECT_TRUE(result.degraded);
    EXPECT_TRUE(safety.is_startup_ok());
    EXPECT_TRUE(safety.is_startup_degraded());

    safety.kick_watchdog();
    EXPECT_FALSE(safety.is_safe());
}

// Test 3: ToF comes back online after startup → run_once() recovers.
//          FAILS because Safety's ping_tof_sensor_impl() doesn't
//          delegate to RangingMock, so the ping-result change is invisible.
TEST(SafetyToFDelegationTest, TryPingRecoveryInRunOnce) {
    StrictMock<MockFireOutput> fire_output;
    EXPECT_CALL(fire_output, fire(false)).Times(AtLeast(1));

    RangingMock ranging;
    ranging.set_mock_ping_result(false);

    Safety safety(nullptr, &ranging, &fire_output);

    // Startup: ToF dead → degraded
    auto result = safety.do_startup_check();
    EXPECT_TRUE(result.degraded);
    EXPECT_FALSE(safety.is_safe());

    // Simulate ToF sensor coming back online
    ranging.set_mock_ping_result(true);

    // run_once() should detect recovery and clear the degraded flag.
    // Call multiple times to give the recovery path a chance.
    for (int i = 0; i < 5; ++i) {
        safety.run_once();
    }

    EXPECT_FALSE(safety.is_startup_degraded());

    safety.kick_watchdog();
    EXPECT_TRUE(safety.is_safe());
}

// Test 4: nullptr IRanging → graceful degradation, no crash.
//          Currently passes because ping_tof_sensor_impl() doesn't
//          dereference ranging_.  After refactoring to delegate,
//          this test ensures the null-guard exists.
TEST(SafetyToFDelegationTest, NullRangingGraceful) {
    StrictMock<MockFireOutput> fire_output;
    EXPECT_CALL(fire_output, fire(false)).Times(AtLeast(1));

    StartupResult result;
    ASSERT_NO_THROW({
        Safety safety(nullptr, nullptr, &fire_output);
        result = safety.do_startup_check();
    });

    // nullptr ranging should not crash; system enters degraded mode.
    EXPECT_TRUE(result.degraded);
}

} // namespace
