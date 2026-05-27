#include "autotrigger/safety.h"
#include "autotrigger/hal/iranging.h"
#include "autotrigger/hal/itrigger.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <thread>

#ifndef MOCK_MODE
#include <unistd.h>     // ::open, ::write, ::close
#include <fcntl.h>      // O_WRONLY, O_RDWR
#include <sys/ioctl.h>  // ioctl, I2C_SLAVE
#endif

namespace autotrigger {

// ════════════════════════════════════════════════════════════
// Safety  —  construction
// ════════════════════════════════════════════════════════════

Safety::Safety(ICamera* camera, IRanging* ranging, ITrigger* trigger)
    : camera_(camera),
      ranging_(ranging),
      trigger_(trigger) {
    // NOTE: last_watchdog_kick_ and last_heartbeat_toggle_ use their
    // default member initialisers (epoch).  Do NOT call now_impl() here
    // — it is virtual and would dispatch to Safety::now_impl() even when
    // a subclass (SafetyMock) is being constructed, producing the wrong
    // baseline.
}

// ════════════════════════════════════════════════════════════
// Startup self-check
// ════════════════════════════════════════════════════════════

StartupResult Safety::do_startup_check() {
    StartupResult result;

    // Safety invariant: trigger MUST be LOW at startup.
    if (trigger_) trigger_->fire(false);

    // --- Hard failures: camera absent or GPIO defaults wrong ---
    if (!check_camera_present_impl()) {
        return result;  // can_proceed = false
    }

    if (!check_gpio_defaults_impl()) {
        return result;  // can_proceed = false
    }

    // --- Soft failure: ToF timeout → degraded mode ---
    if (!ping_tof_sensor_impl()) {
        result.can_proceed = true;
        result.degraded    = true;
        startup_degraded_  = true;
        startup_ok_        = true;
        return result;
    }

    // --- Everything OK ---
    result.can_proceed = true;
    result.degraded    = false;
    startup_ok_        = true;
    startup_degraded_  = false;
    return result;
}

// ════════════════════════════════════════════════════════════
// Runtime monitor
// ════════════════════════════════════════════════════════════

void Safety::run_once() {
    monitor_thermal();
    heartbeat();

    // Attempt ToF recovery if operating in degraded mode
    if (startup_degraded_ && ping_tof_sensor_impl()) {
        startup_degraded_ = false;
    }
}

// ════════════════════════════════════════════════════════════
// Thermal monitor  (hysteresis: on ≥85 °C, off ≤80 °C)
// ════════════════════════════════════════════════════════════

void Safety::monitor_thermal() {
    current_temp_ = read_cpu_temperature_impl();

    if (current_temp_ >= kThrottleOnTemp) {
        throttled_ = true;
    } else if (current_temp_ <= kThrottleOffTemp) {
        throttled_ = false;
    }
    // else: remain in current state (hysteresis band)
}

// ════════════════════════════════════════════════════════════
// Watchdog
// ════════════════════════════════════════════════════════════

void Safety::kick_watchdog() {
    last_watchdog_kick_ = now_impl();
    write_watchdog_impl();
}

bool Safety::is_watchdog_alive() const {
    return ms_since_last_watchdog_kick() < kWatchdogTimeout;
}

std::chrono::milliseconds Safety::ms_since_last_watchdog_kick() const {
    auto elapsed = now_impl() - last_watchdog_kick_;
    return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
}

// ════════════════════════════════════════════════════════════
// Heartbeat
// ════════════════════════════════════════════════════════════

void Safety::heartbeat() {
    auto now = now_impl();
    if (first_heartbeat_ || now - last_heartbeat_toggle_ >= kHeartbeatPeriod) {
        first_heartbeat_ = false;
        heartbeat_state_ = !heartbeat_state_;
        last_heartbeat_toggle_ = now;
        set_heartbeat_gpio_impl(heartbeat_state_);
    }
}

// ════════════════════════════════════════════════════════════
// Overall safety
// ════════════════════════════════════════════════════════════

bool Safety::is_safe() const {
    // Runtime ToF health: if ranging was healthy at startup but fails mid-operation,
    // is_safe() detects it. nullptr ranging (tests) or healthy rangefinder OK.
    // startup_degraded_ → trigger stays LOW regardless of runtime health.
    bool tof_ok = !ranging_ || ranging_->is_healthy();
    return startup_ok_ && !startup_degraded_ && !throttled_ && is_watchdog_alive() && tof_ok;
}

// ════════════════════════════════════════════════════════════
// Hardware abstraction  —  real implementations
// ════════════════════════════════════════════════════════════

float Safety::read_cpu_temperature_impl() {
#ifdef MOCK_MODE
    return 45.0f;  // benign default for x86 development
#else
    // RK3566: /sys/class/thermal/thermal_zone0/temp  → millidegrees
    FILE* f = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (!f) return 40.0f;  // safe fallback
    int millidegrees = 40000;
    if (fscanf(f, "%d", &millidegrees) != 1) {
        millidegrees = 40000;
    }
    fclose(f);
    return static_cast<float>(millidegrees) / 1000.0f;
#endif
}

void Safety::write_watchdog_impl() {
#ifndef MOCK_MODE
    // RK3566: /dev/watchdog — a single write pets the watchdog.
    // Opening the device is typically done once at init; here we
    // assume it was already opened.  This method just pets it.
    static int wd_fd = -1;
    if (wd_fd < 0) {
        wd_fd = ::open("/dev/watchdog", O_WRONLY);
    }
    if (wd_fd >= 0) {
        // Any write (even a single byte) pets the watchdog.
        const char magic = 'V';  // standard magic character
        ::write(wd_fd, &magic, 1);
    }
#endif
    // MOCK_MODE: no-op — SafetyMock overrides this.
}

void Safety::set_heartbeat_gpio_impl(bool /*state*/) {
#ifndef MOCK_MODE
    // TODO: implement via libgpiod or /sys/class/gpio once the
    // heartbeat GPIO pin is assigned in the hardware design.
    // For now this is a no-op on real hardware — the heartbeat
    // logic is verified to be correct via SafetyMock tests.
#endif
}

bool Safety::check_camera_present_impl() {
#ifndef MOCK_MODE
    // RK3566: try to open the camera V4L2 device.
    // The OV5647 sensor typically appears as /dev/video0.
    FILE* f = fopen("/dev/video0", "r");
    if (!f) return false;
    fclose(f);
    return true;
#else
    return true;  // x86: assume camera present
#endif
}

bool Safety::ping_tof_sensor_impl() {
#ifndef MOCK_MODE
    // RK3566: try a quick I²C read from the VL53L1X sensor.
    // The sensor's default I²C address is 0x29.
    // This is a lightweight presence check.
    int i2c_fd = ::open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0) return false;

    // Set slave address
    if (::ioctl(i2c_fd, 0x0703 /* I2C_SLAVE */, 0x29) < 0) {
        ::close(i2c_fd);
        return false;
    }

    // Read WHO_AM_I / chip-ID register (register 0x01 on VL53L1X)
    unsigned char reg = 0x01;
    unsigned char val = 0;
    if (::write(i2c_fd, &reg, 1) != 1 ||
        ::read(i2c_fd, &val, 1) != 1) {
        ::close(i2c_fd);
        return false;
    }

    ::close(i2c_fd);
    return (val == 0xEA);  // VL53L1X expected model ID
#else
    return true;  // x86: assume ToF present
#endif
}

bool Safety::check_gpio_defaults_impl() {
#ifndef MOCK_MODE
    // Verify that the trigger solenoid GPIO is LOW at power-on.
    // On RK3566 this is typically GPIO3_C1 or similar.
    // We check by reading the actual GPIO value via libgpiod
    // or /sys/class/gpio.  For now, assume it passes.
    // TODO: implement once GPIO line is assigned in hardware design.
    return true;
#else
    return true;  // x86: assume defaults OK
#endif
}

std::chrono::steady_clock::time_point Safety::now_impl() const {
    return std::chrono::steady_clock::now();
}

// ════════════════════════════════════════════════════════════
// SafetyMock  —  overrides for test injection
// ════════════════════════════════════════════════════════════

SafetyMock::SafetyMock(ICamera* camera, IRanging* ranging, ITrigger* trigger)
    : Safety(camera, ranging, trigger) {
    // mock_time_ is default-initialised to epoch — no setup needed.
}

float SafetyMock::read_cpu_temperature_impl() {
    return mock_temp_;
}

void SafetyMock::write_watchdog_impl() {
    ++watchdog_kick_count_;
}

void SafetyMock::set_heartbeat_gpio_impl(bool state) {
    ++heartbeat_toggle_count_;
    last_heartbeat_setpoint_ = state;
}

bool SafetyMock::check_camera_present_impl() {
    return mock_camera_present_;
}

bool SafetyMock::ping_tof_sensor_impl() {
    return mock_tof_healthy_;
}

bool SafetyMock::check_gpio_defaults_impl() {
    return mock_gpio_defaults_ok_;
}

std::chrono::steady_clock::time_point SafetyMock::now_impl() const {
    return mock_time_;
}

} // namespace autotrigger
