#pragma once

#include "autotrigger/hal/idisplay.h"
#include <cstdint>

namespace autotrigger {

class DisplayMock : public IDisplay {
public:
    bool init() override;
    void render(int aim_x, int aim_y) override;
    void release() override;

    uint8_t get_pixel(int x, int y) const;

    static constexpr int WIDTH = 240;
    static constexpr int HEIGHT = 240;
    static constexpr int RADIUS = 115;
    static constexpr int CX = 120;
    static constexpr int CY = 120;

private:
    uint8_t fb_[HEIGHT][WIDTH];

    void clear();
    void draw_crosshair();
    void draw_aimpoint(int x, int y);
    void apply_round_mask();
};

} // namespace autotrigger
