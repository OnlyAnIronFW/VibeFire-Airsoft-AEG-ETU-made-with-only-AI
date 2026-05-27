#include "autotrigger/trigger.h"

#include <iostream>

#ifndef MOCK_MODE
#include <gpiod.h>
#endif

namespace autotrigger {

// ============================================================
// Trigger — real GPIO-backed implementation
// ============================================================

Trigger::Trigger() : Trigger(PlatformConfig{}, false) {}

Trigger::Trigger(const PlatformConfig& config, bool demo_mode)
    : config_(config), demo_mode_(demo_mode) {}

bool Trigger::init() {
#ifdef MOCK_MODE
    return true;
#else
    if (demo_mode_) {
        // No GPIO needed in demo mode — fire() prints to stdout.
        return true;
    }

    if (!config_.gpio_chip.empty()) {
        gpio_chip_ = gpiod_chip_open_by_name(config_.gpio_chip.c_str());
    }
    if (!gpio_chip_) {
        // Try numeric path as fallback
        gpio_chip_ = gpiod_chip_open((std::string("/dev/") + config_.gpio_chip).c_str());
    }
    if (!gpio_chip_) {
        std::cerr << "Trigger: failed to open GPIO chip " << config_.gpio_chip << "\n";
        return false;
    }

    // Request fire pin as output, initially LOW
    fire_line_ = gpiod_chip_get_line(
        static_cast<gpiod_chip*>(gpio_chip_), config_.fire_pin);
    if (!fire_line_) {
        std::cerr << "Trigger: failed to get fire line " << config_.fire_pin << "\n";
        return false;
    }
    if (gpiod_line_request_output(
            static_cast<gpiod_line*>(fire_line_), "autotrigger-fire", 0) < 0) {
        std::cerr << "Trigger: failed to request fire line as output\n";
        return false;
    }

    // Request trigger pin as input
    trigger_line_ = gpiod_chip_get_line(
        static_cast<gpiod_chip*>(gpio_chip_), config_.trigger_pin);
    if (!trigger_line_) {
        std::cerr << "Trigger: failed to get trigger line " << config_.trigger_pin << "\n";
        return false;
    }
    if (gpiod_line_request_input(
            static_cast<gpiod_line*>(trigger_line_), "autotrigger-trigger") < 0) {
        std::cerr << "Trigger: failed to request trigger line as input\n";
        return false;
    }

    return true;
#endif
}

bool Trigger::is_held() {
#ifdef MOCK_MODE
    return false;
#else
    if (demo_mode_) return false;
    if (!trigger_line_) return false;
    int val = gpiod_line_get_value(static_cast<gpiod_line*>(trigger_line_));
    return val > 0;
#endif
}

void Trigger::fire(bool on) {
#ifdef MOCK_MODE
    (void)on;
#else
    if (demo_mode_) {
        if (on) {
            std::cout << "AUTO FIRE" << std::endl;
        } else {
            std::cout << "HOLD FIRE" << std::endl;
        }
        return;
    }
    if (!fire_line_) return;
    gpiod_line_set_value(static_cast<gpiod_line*>(fire_line_), on ? 1 : 0);
#endif
}

void Trigger::release() {
#ifdef MOCK_MODE
    // No-op on x86
#else
    if (fire_line_) {
        gpiod_line_set_value(static_cast<gpiod_line*>(fire_line_), 0);
        gpiod_line_release(static_cast<gpiod_line*>(fire_line_));
        fire_line_ = nullptr;
    }
    if (trigger_line_) {
        gpiod_line_release(static_cast<gpiod_line*>(trigger_line_));
        trigger_line_ = nullptr;
    }
    if (gpio_chip_) {
        gpiod_chip_close(static_cast<gpiod_chip*>(gpio_chip_));
        gpio_chip_ = nullptr;
    }
#endif
}

void Trigger::update(bool aim_aligned, bool is_held_val) {
    if (aim_aligned) {
        if (consensus_counter_ < kConsensusFrames) {
            consensus_counter_++;
        }
    } else {
        consensus_counter_ = 0;
    }

    if (consensus_counter_ >= kConsensusFrames && is_held_val) {
        fire(true);
    } else {
        fire(false);
    }
}

// ============================================================
// TriggerMock — injectable test double
// ============================================================

void TriggerMock::fire(bool on) {
    last_fire_ = on;
    fire_call_count_++;
}

void TriggerMock::update(bool aim_aligned) {
    if (aim_aligned) {
        if (consensus_counter_ < Trigger::kConsensusFrames) {
            consensus_counter_++;
        }
    } else {
        consensus_counter_ = 0;
    }

    if (consensus_counter_ >= Trigger::kConsensusFrames && is_held()) {
        fire(true);
    } else {
        fire(false);
    }
}

} // namespace autotrigger
