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

void led_boot_begin();
void led_boot_end();
void led_boot_set_pixel(int index, RGB color);
void led_boot_fill(RGB color);
void led_boot_mark_dirty();

uint32_t led_completed_frame_count();

// Call only between led_begin_update() and led_end_update().
void led_record_midi_arrival_unlocked(uint32_t arrival_us);

// Called only from the low-priority metrics task (never from the LED task).
void led_perf_log_snapshot();
