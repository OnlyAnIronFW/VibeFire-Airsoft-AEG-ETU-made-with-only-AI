#include "autotrigger/trigger.h"

namespace autotrigger {

// ============================================================
// Trigger — real GPIO-backed implementation
// ============================================================

bool Trigger::init() {
#ifdef MOCK_MODE
  // x86 dev: no GPIO available — ready as a stub.
  return true;
#else
  // TODO(RK3566): open gpiochip device, request input/output lines
  //   e.g. gpiod_chip_open_by_name("gpiochip0");
  return false;
#endif
}

bool Trigger::is_held() {
#ifdef MOCK_MODE
  // Stub — no physical trigger on x86 development host.
  return false;
#else
  // TODO: gpiod_line_get_value() on the trigger-input line
  return false;
#endif
}

void Trigger::fire(bool on) {
#ifdef MOCK_MODE
  (void)on;  // No-op on x86
#else
  // TODO: gpiod_line_set_value() on the solenoid-output line
#endif
}

void Trigger::release() {
#ifdef MOCK_MODE
  // No-op on x86
#else
  // TODO: gpiod_line_release() for both input and output lines
#endif
}

void Trigger::update(bool aim_aligned) {
  if (aim_aligned) {
    if (consensus_counter_ < kConsensusFrames) {
      consensus_counter_++;
    }
  } else {
    consensus_counter_ = 0;
  }

  if (consensus_counter_ >= kConsensusFrames && is_held()) {
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
