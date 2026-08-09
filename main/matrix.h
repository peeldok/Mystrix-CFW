#pragma once

#include <stdint.h>

void matrix_init();
int drum_pitch_to_led(uint8_t pitch);
int nonlit_address_to_led(uint8_t address);
int matrix_logical_led_to_physical(int logical_index);
int matrix_physical_led_to_logical(int physical_index);
uint8_t matrix_note_at(uint8_t row, uint8_t col);
