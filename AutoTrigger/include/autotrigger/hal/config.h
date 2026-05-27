#ifndef AUTOTRIGGER_HAL_CONFIG_H_
#define AUTOTRIGGER_HAL_CONFIG_H_

#include <string>

namespace autotrigger {

/**
 * @brief Platform configuration holding all hardware paths.
 *
 * Passed by reference to modules that need GPIO, UART, thermal,
 * or camera device paths.  Default values target the RK3566
 * reference design (gpiochip0, /dev/ttyS1, /dev/video0, etc.).
 *
 * This struct has zero dependencies — it can be included anywhere.
 */
struct PlatformConfig {
    /** UART device for the JRT TOF01 laser rangefinder. */
    std::string uart_device     = "/dev/ttyS1";

    /** Name of the GPIO chip (e.g. "gpiochip0" on RK3566). */
    std::string gpio_chip       = "gpiochip0";

    /** GPIO line number for the fire output (solenoid / FCU). */
    unsigned int fire_pin       = 12;

    /** GPIO line number for the physical trigger input switch. */
    unsigned int trigger_pin    = 13;

    /** V4L2 camera device path (OV5647 / IMX219). */
    std::string camera_device   = "/dev/video0";

    /** sysfs thermal zone path for CPU temperature monitoring. */
    std::string thermal_zone    = "/sys/class/thermal/thermal_zone0/temp";

    /** Path to the hardware watchdog device. */
    std::string watchdog_device = "/dev/watchdog";
};

} // namespace autotrigger

#endif // AUTOTRIGGER_HAL_CONFIG_H_
