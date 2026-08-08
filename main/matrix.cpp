#include "matrix.h"
#include "config.h"
#include "midi.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr uint8_t drum_layout[ROW_NUM][COL_NUM] = {
    {64, 65, 66, 67, 96, 97, 98, 99},
    {60, 61, 62, 63, 92, 93, 94, 95},
    {56, 57, 58, 59, 88, 89, 90, 91},
    {52, 53, 54, 55, 84, 85, 86, 87},
    {48, 49, 50, 51, 80, 81, 82, 83},
    {44, 45, 46, 47, 76, 77, 78, 79},
    {40, 41, 42, 43, 72, 73, 74, 75},
    {36, 37, 38, 39, 68, 69, 70, 71}
};

static constexpr uint8_t nonlit_layout[10][10] = {
    {90, 91, 92, 93, 94, 95, 96, 97, 98, 99},
    {80, 81, 82, 83, 84, 85, 86, 87, 88, 89},
    {70, 71, 72, 73, 74, 75, 76, 77, 78, 79},
    {60, 61, 62, 63, 64, 65, 66, 67, 68, 69},
    {50, 51, 52, 53, 54, 55, 56, 57, 58, 59},
    {40, 41, 42, 43, 44, 45, 46, 47, 48, 49},
    {30, 31, 32, 33, 34, 35, 36, 37, 38, 39},
    {20, 21, 22, 23, 24, 25, 26, 27, 28, 29},
    {10, 11, 12, 13, 14, 15, 16, 17, 18, 19},
    { 0,  1,  2,  3,  4,  5,  6,  7,  8,  9}
};

static int8_t drum_lookup[128];
static bool raw_state[ROW_NUM][COL_NUM]{};
static bool stable_state[ROW_NUM][COL_NUM]{};
static int64_t last_change_us[ROW_NUM][COL_NUM]{};


uint8_t matrix_note_at(uint8_t row, uint8_t col) {
    if (row >= ROW_NUM || col >= COL_NUM) return 0;
    return drum_layout[row][col];
}
int drum_pitch_to_led(uint8_t pitch) {
    return pitch < 128 ? drum_lookup[pitch] : -1;
}

int nonlit_address_to_led(uint8_t address) {
    if (address == 0 || address == 9 || address == 90 || address == 99) return -1;

    for (int r = 0; r < 10; ++r) {
        for (int c = 0; c < 10; ++c) {
            if (nonlit_layout[r][c] != address) continue;

            if (r >= 1 && r <= 8 && c >= 1 && c <= 8) {
                return ((r - 1) * 8) + (c - 1);
            }
            if (c == 9 && r >= 1 && r <= 8) {
                return 64 + (8 - r);
            }
            if (r == 0 && c >= 1 && c <= 8) {
                return 72 + (8 - c);
            }
            if (c == 0 && r >= 1 && r <= 8) {
                return 80 + (r - 1);
            }
            if (r == 9 && c >= 1 && c <= 8) {
                return 88 + (c - 1);
            }
        }
    }
    return -1;
}

void matrix_init() {
    for (auto &v : drum_lookup) v = -1;

    for (int r = 0; r < ROW_NUM; ++r) {
        for (int c = 0; c < COL_NUM; ++c) {
            drum_lookup[drum_layout[r][c]] = r * COL_NUM + c;
        }
    }

    for (int i = 0; i < 8; ++i) {
        drum_lookup[107 - i] = 64 + i;
        drum_lookup[35 - i] = 72 + i;
        drum_lookup[108 + i] = 80 + i;
        drum_lookup[116 + i] = 88 + i;
    }
}
