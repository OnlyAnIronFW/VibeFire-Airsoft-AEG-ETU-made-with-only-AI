#ifndef AUTOTRIGGER_HAL_IFIRE_OUTPUT_H_
#define AUTOTRIGGER_HAL_IFIRE_OUTPUT_H_

namespace autotrigger {

/**
 * @brief Hardware abstraction for the fire output signal.
 *
 * Replaces the old ITrigger::fire() — the fire solenoid / FCU
 * output is now a standalone concern, decoupled from the physical
 * trigger-input interface.
 *
 * Implementations write to a GPIO line (libgpiod on ARM64) or
 * print diagnostic messages in demo / mock modes.
 */
class IFireOutput {
public:
    virtual ~IFireOutput() = default;

    /** Initialize the fire output GPIO line. */
    virtual bool init() = 0;

    /**
     * Set the fire output state.
     * @param on  true = energise solenoid (HIGH), false = de-energise (LOW).
     */
    virtual void fire(bool on) = 0;

    /** Release GPIO resources. */
    virtual void release() = 0;
};

} // namespace autotrigger

#endif // AUTOTRIGGER_HAL_IFIRE_OUTPUT_H_
