#pragma once

#include <stdint.h>
#include "types.h"

extern const uint8_t _r[128];
extern const uint8_t _g[128];
extern const uint8_t _b[128];
extern const uint8_t _r2[128];
extern const uint8_t _g2[128];
extern const uint8_t _b2[128];
extern const uint8_t _r3[128];
extern const uint8_t _g3[128];
extern const uint8_t _b3[128];

extern uint8_t g_web_r[3][128];
extern uint8_t g_web_g[3][128];
extern uint8_t g_web_b[3][128];
extern uint8_t g_brightness;

void palette_init();
void palette_save_web(uint8_t palette, uint8_t component, const uint8_t data[128]);
RGB palette_color(uint8_t channel_zero_based, uint8_t velocity);
