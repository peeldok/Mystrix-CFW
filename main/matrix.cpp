#include "matrix.h"
#include "config.h"
#include "midi.h"
#include "settings.h"

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

static void physical_to_logical(uint8_t row, uint8_t col, uint8_t &logical_row, uint8_t &logical_col) {
    switch (settings_rotation()) {
        case RotationMode::Deg90:
            logical_row = col;
            logical_col = static_cast<uint8_t>(7 - row);
            break;
        case RotationMode::Deg180:
            logical_row = static_cast<uint8_t>(7 - row);
            logical_col = static_cast<uint8_t>(7 - col);
            break;
        case RotationMode::Deg270:
            logical_row = static_cast<uint8_t>(7 - col);
            logical_col = row;
            break;
        default:
            logical_row = row;
            logical_col = col;
            break;
    }
}

static bool led_to_extended_coord(int index, int &row, int &col) {
    if (index >= 0 && index < 64) {
        row = index / 8 + 1;
        col = index % 8 + 1;
        return true;
    }

    if (index >= 64 && index < 72) {
        const int i = index - 64;
        row = 8 - i;
        col = 9;
        return true;
    }

    if (index >= 72 && index < 80) {
        const int i = index - 72;
        row = 0;
        col = 8 - i;
        return true;
    }

    if (index >= 80 && index < 88) {
        const int i = index - 80;
        row = i + 1;
        col = 0;
        return true;
    }

    if (index >= 88 && index < 96) {
        const int i = index - 88;
        row = 9;
        col = i + 1;
        return true;
    }

    return false;
}

static int extended_coord_to_led(int row, int col) {
    if (row >= 1 && row <= 8 && col >= 1 && col <= 8) {
        return (row - 1) * 8 + (col - 1);
    }
    if (col == 9 && row >= 1 && row <= 8) {
        return 64 + (8 - row);
    }
    if (row == 0 && col >= 1 && col <= 8) {
        return 72 + (8 - col);
    }
    if (col == 0 && row >= 1 && row <= 8) {
        return 80 + (row - 1);
    }
    if (row == 9 && col >= 1 && col <= 8) {
        return 88 + (col - 1);
    }
    return -1;
}

uint8_t matrix_note_at(uint8_t row, uint8_t col) {
    if (row >= ROW_NUM || col >= COL_NUM) return 0;

    uint8_t logical_row = row;
    uint8_t logical_col = col;
    physical_to_logical(row, col, logical_row, logical_col);
    return drum_layout[logical_row][logical_col];
}

int drum_pitch_to_led(uint8_t pitch) {
    return pitch < 128 ? drum_lookup[pitch] : -1;
}

int nonlit_address_to_led(uint8_t address) {
    if (address == 0 || address == 9 || address == 90 || address == 99) return -1;

    for (int r = 0; r < 10; ++r) {
        for (int c = 0; c < 10; ++c) {
            if (nonlit_layout[r][c] != address) continue;
            return extended_coord_to_led(r, c);
        }
    }
    return -1;
}

int matrix_logical_led_to_physical(int logical_index) {
    int logical_row = 0;
    int logical_col = 0;
    if (!led_to_extended_coord(logical_index, logical_row, logical_col)) return -1;

    int physical_row = logical_row;
    int physical_col = logical_col;

    switch (settings_rotation()) {
        case RotationMode::Deg90:
            physical_row = 9 - logical_col;
            physical_col = logical_row;
            break;
        case RotationMode::Deg180:
            physical_row = 9 - logical_row;
            physical_col = 9 - logical_col;
            break;
        case RotationMode::Deg270:
            physical_row = logical_col;
            physical_col = 9 - logical_row;
            break;
        default:
            break;
    }

    return extended_coord_to_led(physical_row, physical_col);
}

int matrix_physical_led_to_logical(int physical_index) {
    if (physical_index < 0 || physical_index >= 64) return -1;

    const uint8_t physical_row = static_cast<uint8_t>(physical_index / 8);
    const uint8_t physical_col = static_cast<uint8_t>(physical_index % 8);
    uint8_t logical_row = physical_row;
    uint8_t logical_col = physical_col;
    physical_to_logical(physical_row, physical_col, logical_row, logical_col);
    return logical_row * 8 + logical_col;
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
