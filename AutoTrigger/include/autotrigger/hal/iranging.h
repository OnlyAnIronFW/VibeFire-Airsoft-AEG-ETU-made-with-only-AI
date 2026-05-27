#pragma once

namespace autotrigger {

/**
 * @brief Hardware abstraction interface for laser ranging sensor.
 *
 * Implementations must provide distance measurement via VL53L1X
 * ToF sensor over I2C, plus health monitoring.
 */
class IRanging {
public:
  virtual ~IRanging() = default;

  /** Initialize the ranging sensor. */
  virtual bool init() = 0;

  /** Return the current distance measurement in metres. */
  virtual float get_distance() = 0;

  /** Check whether the sensor is responding normally. */
  virtual bool is_healthy() = 0;
};

} // namespace autotrigger
