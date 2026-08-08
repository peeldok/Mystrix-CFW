#pragma once

#include <stdint.h>

enum class SettingsPage : uint8_t {
    Main = 0,
    Brightness = 1,
};

enum class BrightnessTarget : uint8_t {
    Matrix = 0,
    Side = 1,
};

bool settings_init();
bool settings_ui_active();
bool settings_velocity_enabled();
SettingsPage settings_page();
BrightnessTarget settings_brightness_target();
uint8_t settings_matrix_brightness();
uint8_t settings_side_brightness();
uint8_t settings_selected_brightness();
uint8_t settings_selected_brightness_step();

void settings_toggle_ui();
void settings_toggle_velocity();
void settings_open_brightness();
void settings_select_brightness_target(BrightnessTarget target);
void settings_set_brightness_step(uint8_t step);
void settings_process();
void settings_enter_tinyuf2();
