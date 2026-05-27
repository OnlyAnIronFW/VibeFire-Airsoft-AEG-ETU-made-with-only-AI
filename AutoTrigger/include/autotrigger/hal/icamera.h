#pragma once

#include <opencv2/core/mat.hpp>

namespace autotrigger {

/**
 * @brief Hardware abstraction interface for camera sensor.
 *
 * Implementations must provide the OV5647 sensor initialization,
 * frame capture pipeline, and resource cleanup.
 */
class ICamera {
public:
  virtual ~ICamera() = default;

  /** Initialize the camera and configure capture parameters. */
  virtual bool init() = 0;

  /** Capture a single frame from the camera. */
  virtual cv::Mat capture() = 0;

  /** Release camera resources. */
  virtual void release() = 0;
};

} // namespace autotrigger
