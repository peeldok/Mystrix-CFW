#pragma once

#include <cstdint>

void fsr_init();
void fsr_start();

uint8_t fsr_min_velocity();
uint8_t fsr_max_velocity();
uint32_t fsr_min_slope();
uint32_t fsr_max_slope();
void fsr_set_velocity_config(uint8_t min_velocity, uint8_t max_velocity, uint32_t min_slope, uint32_t max_slope);
