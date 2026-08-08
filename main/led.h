#pragma once

#include <stdint.h>
#include "types.h"
#include "settings.h"

void led_init();
void led_set_pixel(int index, RGB color);
void led_fill(RGB color);
void led_set_brightness(uint8_t brightness);
void led_mark_dirty();

void led_begin_update();
void led_set_pixel_unlocked(int index, RGB color);
void led_fill_unlocked(RGB color);
void led_end_update(bool notify = true);

void led_set_settings_ui(
    bool active,
    bool velocity_enabled,
    SettingsPage page,
    BrightnessTarget target,
    uint8_t matrix_brightness,
    uint8_t side_brightness
);
