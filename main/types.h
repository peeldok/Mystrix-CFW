#pragma once

#include <stdint.h>

struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct ButtonEvent {
    uint8_t note;
    bool pressed;
    uint8_t velocity;
};
