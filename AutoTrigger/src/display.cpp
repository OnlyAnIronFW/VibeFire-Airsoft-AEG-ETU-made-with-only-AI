#include "autotrigger/display.h"
#include <algorithm>
#include <cstring>

namespace autotrigger {

bool DisplayMock::init() {
    clear();
    return true;
}

void DisplayMock::release() {
    clear();
}

void DisplayMock::clear() {
    std::memset(fb_, 0, sizeof(fb_));
}

uint8_t DisplayMock::get_pixel(int x, int y) const {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return 0;
    return fb_[y][x];
}

void DisplayMock::draw_crosshair() {
    // Horizontal line: 2px thick at y = CY-1, CY (119, 120)
    for (int x = 0; x < WIDTH; ++x) {
        fb_[CY - 1][x] = 255;
        fb_[CY][x] = 255;
    }
    // Vertical line: 2px thick at x = CX-1, CX (119, 120)
    for (int y = 0; y < HEIGHT; ++y) {
        fb_[y][CX - 1] = 255;
        fb_[y][CX] = 255;
    }
}

void DisplayMock::draw_aimpoint(int ax, int ay) {
    // Clamp to screen bounds
    int x = std::clamp(ax, 0, WIDTH - 1);
    int y = std::clamp(ay, 0, HEIGHT - 1);

    // Filled circle, radius 5
    constexpr int R2 = 5 * 5; // radius squared
    for (int dy = -5; dy <= 5; ++dy) {
        for (int dx = -5; dx <= 5; ++dx) {
            if (dx * dx + dy * dy <= R2) {
                int px = x + dx;
                int py = y + dy;
                if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
                    fb_[py][px] = 255;
                }
            }
        }
    }
}

void DisplayMock::apply_round_mask() {
    constexpr int R2 = RADIUS * RADIUS;
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            int dx = x - CX;
            int dy = y - CY;
            if (dx * dx + dy * dy > R2) {
                fb_[y][x] = 0;
            }
        }
    }
}

void DisplayMock::render(int aim_x, int aim_y) {
    clear();
    draw_crosshair();
    draw_aimpoint(aim_x, aim_y);
    apply_round_mask();
}

} // namespace autotrigger
