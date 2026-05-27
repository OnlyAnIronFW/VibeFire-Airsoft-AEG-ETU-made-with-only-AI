#pragma once

namespace autotrigger {

/**
 * @brief Hardware abstraction interface for laser ranging sensor.
 *
 * Implementations provide distance measurement and sensor health
 * monitoring for ToF laser rangefinders (e.g., JRT TOF01 UART).
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

  /** Attempt to contact the sensor and verify it is alive.
   *  @param timeout_ms  Maximum wait time in milliseconds (default 500).
   *  @return true if the sensor responds with valid data within timeout. */
  virtual bool try_ping(int timeout_ms = 500) = 0;
};

} // namespace autotrigger
