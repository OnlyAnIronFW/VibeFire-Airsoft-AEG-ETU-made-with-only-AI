#pragma once

namespace autotrigger {

/**
 * @brief Hardware abstraction interface for the trigger/solenoid.
 *
 * Implementations must provide trigger-hold detection
 * and fire-signal output via GPIO (libgpiod on ARM64).
 */
class ITrigger {
public:
  virtual ~ITrigger() = default;

  /** Initialize the trigger subsystem. */
  virtual bool init() = 0;

  /** Return true while the physical trigger is pressed. */
  virtual bool is_held() = 0;

  /**
   * Set the fire solenoid state.
   * @param on  true = energise solenoid, false = de-energise.
   */
  virtual void fire(bool on) = 0;

  /** Release GPIO resources. */
  virtual void release() = 0;
};

} // namespace autotrigger
