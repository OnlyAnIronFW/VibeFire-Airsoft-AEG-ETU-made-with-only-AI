#ifndef AUTOTRIGGER_TRIGGER_H_
#define AUTOTRIGGER_TRIGGER_H_

#include "autotrigger/hal/config.h"
#include "autotrigger/hal/ifire_output.h"
#include "autotrigger/hal/itrigger.h"

namespace autotrigger {

/// Number of consecutive aligned frames required before firing.
inline constexpr int kTriggerConsensusFrames = 3;

// ──────────────────────────────────────────────
// Trigger — real GPIO-backed implementation
// ──────────────────────────────────────────────
//
// ARM64 (RK3566): uses libgpiod for GPIO read/write.
// x86 (MOCK_MODE): stubbed — is_held() returns false, fire() is no-op.
//
// Trigger implements BOTH ITrigger (physical trigger input) and
// IFireOutput (solenoid/FCU output), separating the two concerns
// that were previously conflated in the old ITrigger interface.
//
// The 3-frame consensus ensures the solenoid is only energised
// when both the physical trigger is held AND the aim-point has
// been within threshold for 3 consecutive frames.
// Safety: output defaults LOW and drops immediately on aim loss.
// ──────────────────────────────────────────────
class Trigger : public ITrigger, public IFireOutput {
public:
    Trigger();
    explicit Trigger(const PlatformConfig& config, bool demo_mode = false);

    // ── ITrigger (input) ──────────────────────
    bool init() override;
    bool is_held() override;
    void release() override;

    // ── IFireOutput (output) ──────────────────
    void fire(bool on) override;

    /// Called every frame. Implements 3-frame aim consensus.
    /// @param aim_aligned  true if the aim-point is within threshold
    /// @param is_held_val  result of ITrigger::is_held() from the caller
    void update(bool aim_aligned, bool is_held_val);

    /// Number of consecutive aligned frames required before firing.
    static constexpr int kConsensusFrames = kTriggerConsensusFrames;

protected:
    PlatformConfig config_;
    bool demo_mode_;

private:
    int consensus_counter_ = 0;

#ifdef MOCK_MODE
    // MOCK_MODE: no GPIO handles needed
#else
    // libgpiod handles (nullptr when demo_mode or unavailable)
    void* gpio_chip_ = nullptr;
    void* fire_line_ = nullptr;
    void* trigger_line_ = nullptr;
#endif
};

// ──────────────────────────────────────────────
// TriggerMock — injectable test double
// ──────────────────────────────────────────────
//
// Tests set inject_held_ to simulate the physical trigger,
// then call update() and verify last_fire_ / fire_call_count_.
//
// Implements both ITrigger and IFireOutput so it can be
// substituted wherever either interface is expected.
// ──────────────────────────────────────────────
class TriggerMock : public ITrigger, public IFireOutput {
public:
    /// Inject: set true to simulate trigger pressed.
    bool inject_held_ = false;

    /// Observe: last value passed to fire().
    bool last_fire_ = false;

    /// Observe: total fire() calls (for call-count assertions).
    int fire_call_count_ = 0;

    bool init() override { return true; }
    bool is_held() override { return inject_held_; }
    void fire(bool on) override;
    void release() override {}

    /// Single-parameter update for test convenience.
    /// Internally calls is_held() (reads inject_held_).
    void update(bool aim_aligned);

private:
    int consensus_counter_ = 0;
};

} // namespace autotrigger

#endif // AUTOTRIGGER_TRIGGER_H_
