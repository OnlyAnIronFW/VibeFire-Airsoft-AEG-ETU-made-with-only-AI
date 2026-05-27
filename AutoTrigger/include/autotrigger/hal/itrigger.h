#ifndef AUTOTRIGGER_HAL_ITRIGGER_H_
#define AUTOTRIGGER_HAL_ITRIGGER_H_

namespace autotrigger {

/**
 * @brief Hardware abstraction for the physical trigger input.
 *
 * Implementations must detect the state of the physical pull-switch
 * via GPIO (libgpiod on ARM64).
 *
 * The fire output has been moved to IFireOutput — this interface
 * is now input-only: init, is_held, release.
 */
class ITrigger {
public:
    virtual ~ITrigger() = default;

    /** Initialize the trigger input GPIO line. */
    virtual bool init() = 0;

    /** Return true while the physical trigger is pressed. */
    virtual bool is_held() = 0;

    /** Release GPIO resources. */
    virtual void release() = 0;
};

} // namespace autotrigger

#endif // AUTOTRIGGER_HAL_ITRIGGER_H_
