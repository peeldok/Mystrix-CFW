#pragma once

#include <stdint.h>

void matrix_init();
int drum_pitch_to_led(uint8_t pitch);
int nonlit_address_to_led(uint8_t address);

uint8_t matrix_note_at(uint8_t row, uint8_t col);
