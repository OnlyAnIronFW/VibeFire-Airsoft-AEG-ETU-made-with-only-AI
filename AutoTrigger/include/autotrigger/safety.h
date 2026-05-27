#pragma once

#include <chrono>
#include <cstdint>

namespace autotrigger {

// Forward declarations — full HAL headers include OpenCV/Eigen
// and are pulled in by safety.cpp only.
class ICamera;
class IRanging;
class ITrigger;

// ────────────────────────────────────────────────────────────
// StartupResult
// ────────────────────────────────────────────────────────────

/// Returned by Safety::do_startup_check() — distinguishes hard
/// failure (cannot proceed) from degraded mode (ToF unavailable).
struct StartupResult {
    bool can_proceed = false;   ///< System may continue booting
    bool degraded     = false;  ///< Running w/o ToF — trigger stays LOW
};

// ────────────────────────────────────────────────────────────
// Safety
// ────────────────────────────────────────────────────────────

/**
 * @brief Central safety monitor for the AutoTrigger system.
 *
 * Responsibilities:
 *   - Startup self-check (camera present, ToF ping, GPIO defaults)
 *   - CPU thermal monitoring with hysteresis
 *   - External watchdog kicking (/dev/watchdog)
 *   - Heartbeat LED toggling (1 Hz)
 *
 * All hardware-access methods are virtual protected so that
 * SafetyMock (below) can inject controlled values in x86 tests.
 *
 * On RK3566 the trigger output MUST be LOW at startup and
 * remain LOW during any fault condition.
 */
class Safety {
public:
    Safety(ICamera* camera, IRanging* ranging, ITrigger* trigger);
    virtual ~Safety() = default;

    // ── Startup ──────────────────────────────────────────────

    /** Run power-on self-test. Returns can_proceed + degraded flags.
     *  Calls trigger->fire(false) to guarantee LOW at startup. */
    StartupResult do_startup_check();

    /** True after a successful startup check. */
    bool is_startup_ok() const { return startup_ok_; }

    /** True when the ToF sensor was absent at startup. */
    bool is_startup_degraded() const { return startup_degraded_; }

    // ── Runtime monitor (call from main loop at ~20–50 Hz) ───

    /** One tick of the safety monitor: thermal, heartbeat, ToF recovery. */
    void run_once();

    // ── Thermal ──────────────────────────────────────────────

    /** Read CPU temperature and update throttled flag (hysteresis). */
    void monitor_thermal();

    /** Most recently read CPU temperature (°C). */
    float current_temperature() const { return current_temp_; }

    /** True when CPU temp exceeded kThrottleOnTemp and hasn't
     *  dropped below kThrottleOffTemp yet. */
    bool is_throttled() const { return throttled_; }

    // ── Watchdog ─────────────────────────────────────────────

    /** Pet the external watchdog. Must be called every <100 ms. */
    void kick_watchdog();

    /** True if kick_watchdog() was called within kWatchdogTimeout. */
    bool is_watchdog_alive() const;

    /** Milliseconds elapsed since the last kick_watchdog() call. */
    std::chrono::milliseconds ms_since_last_watchdog_kick() const;

    // ── Heartbeat ────────────────────────────────────────────

    /** Toggle the heartbeat LED at 1 Hz. Call at ~20–50 Hz. */
    void heartbeat();

    /** Current heartbeat GPIO state (true = ON). */
    bool heartbeat_state() const { return heartbeat_state_; }

    // ── Overall safety ───────────────────────────────────────

    /** System is safe to fire: startup OK, not degraded,
     *  not throttled, watchdog alive. */
    bool is_safe() const;

    // ── Thresholds (exposed for test readability) ─────────────

    static constexpr float kThrottleOnTemp  = 85.0f;  ///< °C
    static constexpr float kThrottleOffTemp = 80.0f;  ///< °C (hysteresis)

    static constexpr std::chrono::milliseconds kWatchdogTimeout{100}; ///< ms
    static constexpr std::chrono::milliseconds kHeartbeatPeriod{500}; ///< ms (→ 1 Hz)

protected:
    // ── Hardware abstraction layer (virtual for mocking) ──────

    /** Read CPU temperature from /sys/class/thermal/thermal_zone0/temp.
     *  Returns degrees Celsius. */
    virtual float read_cpu_temperature_impl();

    /** Pet the hardware watchdog (/dev/watchdog). */
    virtual void write_watchdog_impl();

    /** Set heartbeat GPIO output (true = on / false = off). */
    virtual void set_heartbeat_gpio_impl(bool state);

    /** Check whether the camera sensor is present and responsive. */
    virtual bool check_camera_present_impl();

    /** Ping the ToF ranging sensor. Returns true if it responds. */
    virtual bool ping_tof_sensor_impl();

    /** Verify that trigger GPIO defaults to LOW at power-on. */
    virtual bool check_gpio_defaults_impl();

    /** Clock source — overridable for time-controlled tests. */
    virtual std::chrono::steady_clock::time_point now_impl() const;

    // ── Members accessible by SafetyMock ─────────────────────

    ICamera*  camera_;
    IRanging* ranging_;
    ITrigger* trigger_;

    bool startup_ok_        = false;
    bool startup_degraded_  = false;
    bool throttled_         = false;
    bool heartbeat_state_   = false;
    bool first_heartbeat_   = true;  ///< forces toggle on first call
    float current_temp_     = 40.0f;

    // Initialised to epoch — no virtual call in constructor.
    // Watchdog starts expired (elapsed ≫ 100ms on real hardware).
    std::chrono::steady_clock::time_point last_watchdog_kick_{};
    std::chrono::steady_clock::time_point last_heartbeat_toggle_{};
};

// ────────────────────────────────────────────────────────────
// SafetyMock  —  test double with injectable sensor values
// ────────────────────────────────────────────────────────────

/**
 * @brief Safety subclass for x86 testing.
 *
 * Overrides every hardware-access method so that no real
 * /dev/watchdog, /sys/class/thermal, GPIO, or I²C hardware
 * is touched.  Use the set_* methods to inject sensor values
 * and the query methods to inspect side-effects.
 *
 * Time control:
 *   SafetyMock replaces now_impl() with a synthetic clock.
 *   Call advance_time_ms() to move the clock forward.
 */
class SafetyMock : public Safety {
public:
    SafetyMock(ICamera* camera, IRanging* ranging, ITrigger* trigger);

    // ── Sensor injection ─────────────────────────────────────

    void set_mock_temperature(float t)        { mock_temp_ = t; }
    void set_mock_camera_present(bool p)      { mock_camera_present_ = p; }
    void set_mock_tof_healthy(bool h)         { mock_tof_healthy_ = h; }
    void set_mock_gpio_defaults_ok(bool g)    { mock_gpio_defaults_ok_ = g; }

    // ── State inspection ─────────────────────────────────────

    /** Number of times write_watchdog_impl() was called. */
    int  watchdog_kick_count()     const { return watchdog_kick_count_; }

    /** Number of times set_heartbeat_gpio_impl() was called. */
    int  heartbeat_toggle_count()  const { return heartbeat_toggle_count_; }

    /** Last value passed to set_heartbeat_gpio_impl(). */
    bool last_heartbeat_setpoint() const { return last_heartbeat_setpoint_; }

    // ── Synthetic time control ───────────────────────────────

    /** Advance the mock clock by `ms` milliseconds. */
    void advance_time_ms(uint32_t ms) {
        mock_time_ += std::chrono::milliseconds(ms);
    }

    /** Reset mock clock to a known baseline. */
    void reset_time() {
        mock_time_ = std::chrono::steady_clock::time_point{};
    }

protected:
    float read_cpu_temperature_impl() override;
    void write_watchdog_impl() override;
    void set_heartbeat_gpio_impl(bool state) override;
    bool check_camera_present_impl() override;
    bool ping_tof_sensor_impl() override;
    bool check_gpio_defaults_impl() override;
    std::chrono::steady_clock::time_point now_impl() const override;

private:
    float  mock_temp_            = 45.0f;
    bool   mock_camera_present_  = true;
    bool   mock_tof_healthy_     = true;
    bool   mock_gpio_defaults_ok_ = true;

    int    watchdog_kick_count_        = 0;
    int    heartbeat_toggle_count_     = 0;
    bool   last_heartbeat_setpoint_    = false;

    std::chrono::steady_clock::time_point mock_time_{};
};

} // namespace autotrigger
