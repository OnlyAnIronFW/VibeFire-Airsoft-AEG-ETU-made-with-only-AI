#pragma once

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
// The 3-frame consensus ensures the solenoid is only energised
// when both the physical trigger is held AND the aim-point has
// been within threshold for 3 consecutive frames.
// Safety: output defaults LOW and drops immediately on aim loss.
// ──────────────────────────────────────────────
class Trigger : public ITrigger {
public:
  bool init() override;
  bool is_held() override;
  void fire(bool on) override;
  void release() override;

  /// Called every frame. Implements 3-frame aim consensus.
  /// @param aim_aligned  true if the aim-point is within threshold
  void update(bool aim_aligned);

  /// Number of consecutive aligned frames required before firing.
  static constexpr int kConsensusFrames = kTriggerConsensusFrames;

private:
  int consensus_counter_ = 0;
};

// ──────────────────────────────────────────────
// TriggerMock — injectable test double
// ──────────────────────────────────────────────
//
// Tests set inject_held_ to simulate the physical trigger,
// then call update() and verify last_fire_ / fire_call_count_.
// ──────────────────────────────────────────────
class TriggerMock : public ITrigger {
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

  void update(bool aim_aligned);

private:
  int consensus_counter_ = 0;
};

} // namespace autotrigger
