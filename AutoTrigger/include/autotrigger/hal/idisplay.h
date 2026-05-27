#pragma once

namespace autotrigger {

/**
 * @brief Hardware abstraction interface for the LCD display.
 *
 * Implementations must provide SDL2-based rendering
 * of the aiming reticle and status overlay.
 */
class IDisplay {
public:
  virtual ~IDisplay() = default;

  /** Initialize the display subsystem. */
  virtual bool init() = 0;

  /**
   * Render a single frame.
   * @param aim_x  X coordinate of the aiming point, in pixels.
   * @param aim_y  Y coordinate of the aiming point, in pixels.
   */
  virtual void render(int aim_x, int aim_y) = 0;

  /** Release display resources. */
  virtual void release() = 0;
};

} // namespace autotrigger
