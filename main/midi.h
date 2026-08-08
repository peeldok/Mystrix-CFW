#pragma once
#include <stdint.h>

void midi_init();
void midi_queue_button(uint8_t note, bool pressed, uint8_t velocity = 127);
