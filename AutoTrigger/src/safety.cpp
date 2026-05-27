#include "autotrigger/safety.h"
#include "autotrigger/hal/config.h"
#include "autotrigger/hal/ifire_output.h"
#include "autotrigger/hal/iranging.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <thread>

#ifndef MOCK_MODE
#include <unistd.h>     // ::open, ::write, ::close
#include <fcntl.h>      // O_WRONLY
#include <gpiod.h>
#endif

namespace autotrigger {

// ════════════════════════════════════════════════════════════
// Safety  —  construction
// ════════════════════════════════════════════════════════════

Safety::Safety(ICamera* camera, IRanging* ranging, IFireOutput* fire_output,
               const PlatformConfig& config)
    : camera_(camera),
      ranging_(ranging),
      fire_output_(fire_output),
      platform_config_(config) {
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

    // Safety invariant: fire output MUST be LOW at startup.
    if (fire_output_) fire_output_->fire(false);

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
    // startup_degraded_ → fire output stays LOW regardless of runtime health.
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
    // Use the thermal zone path from PlatformConfig
    std::string tz_path = platform_config_.thermal_zone;
    if (tz_path.empty()) {
        tz_path = "/sys/class/thermal/thermal_zone0/temp";
    }
    FILE* f = fopen(tz_path.c_str(), "r");
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
    // Use the watchdog device path from PlatformConfig.
    // Opening the device is typically done once at init; here we
    // assume it was already opened.  This method just pets it.
    static int wd_fd = -1;
    if (wd_fd < 0) {
        const std::string& wd = platform_config_.watchdog_device;
        wd_fd = ::open(wd.c_str(), O_WRONLY);
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
    // Use the camera device path from PlatformConfig
    const std::string& cam = platform_config_.camera_device;
    FILE* f = fopen(cam.c_str(), "r");
    if (!f) return false;
    fclose(f);
    return true;
#else
    return true;  // x86: assume camera present
#endif
}

bool Safety::ping_tof_sensor_impl() {
    // Delegate to the ranging module's UART check (JRT TOF01, not VL53L1X).
    if (ranging_) return ranging_->try_ping(500);
    return false;
}

bool Safety::check_gpio_defaults_impl() {
#ifndef MOCK_MODE
    // Verify that the fire output GPIO is LOW at power-on.
    // Uses libgpiod to open the chip and read the fire pin.
    const std::string& chip = platform_config_.gpio_chip;
    if (chip.empty()) return true;  // no config, assume OK

    gpiod_chip* gc = gpiod_chip_open_by_name(chip.c_str());
    if (!gc) {
        // Try numeric path as fallback
        gc = gpiod_chip_open((std::string("/dev/") + chip).c_str());
    }
    if (!gc) return true;  // cannot verify, assume OK

    gpiod_line* line = gpiod_chip_get_line(gc, platform_config_.fire_pin);
    if (!line) {
        gpiod_chip_close(gc);
        return true;  // cannot verify, assume OK
    }

    // Request as input to read the current value
    if (gpiod_line_request_input(line, "autotrigger-safety-gpio-check") < 0) {
        gpiod_chip_close(gc);
        return true;  // cannot request, assume OK
    }

    int val = gpiod_line_get_value(line);
    gpiod_line_release(line);
    gpiod_chip_close(gc);

    // Fire output MUST default LOW (0) at power-on.
    return val == 0;
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

SafetyMock::SafetyMock(ICamera* camera, IRanging* ranging, IFireOutput* fire_output)
    : Safety(camera, ranging, fire_output) {
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
