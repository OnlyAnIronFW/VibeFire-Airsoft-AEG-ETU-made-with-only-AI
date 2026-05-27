#include "autotrigger/hal/config.h"

#include <fstream>
#include <iostream>
#include <string>

namespace autotrigger {

/**
 * @brief Load PlatformConfig from a simple key=value file.
 *
 * Lines beginning with '#' are comments.  Blank lines are ignored.
 *
 * Supported keys: uart_device, gpio_chip, fire_pin, trigger_pin,
 *                 camera_device, thermal_zone, watchdog_device
 */
PlatformConfig platform_config_load(const std::string& path) {
    PlatformConfig config;
    std::ifstream in(path);
    if (!in.is_open()) {
        std::cerr << "platform_config_load: cannot open " << path << "\n";
        return config;
    }
    std::string line;
    while (std::getline(in, line)) {
        auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) line = line.substr(0, comment_pos);

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        // Trim whitespace
        key.erase(0, key.find_first_not_of(" \t\r\n"));
        key.erase(key.find_last_not_of(" \t\r\n") + 1);
        val.erase(0, val.find_first_not_of(" \t\r\n"));
        val.erase(val.find_last_not_of(" \t\r\n") + 1);

        if (key.empty() || val.empty()) continue;

        if (key == "uart_device")
            config.uart_device = val;
        else if (key == "gpio_chip")
            config.gpio_chip = val;
        else if (key == "fire_pin")
            config.fire_pin = static_cast<unsigned int>(std::stoul(val));
        else if (key == "trigger_pin")
            config.trigger_pin = static_cast<unsigned int>(std::stoul(val));
        else if (key == "camera_device")
            config.camera_device = val;
        else if (key == "thermal_zone")
            config.thermal_zone = val;
        else if (key == "watchdog_device")
            config.watchdog_device = val;
    }
    return config;
}

} // namespace autotrigger
