#pragma once

#include <stdint.h>

static constexpr int LED_GPIO = 38;
static constexpr int FN_GPIO = 16;
static constexpr int NUM_LEDS = 96;

static constexpr int ROW_NUM = 8;
static constexpr int COL_NUM = 8;

static constexpr int ROW_PINS[ROW_NUM] = {2, 3, 4, 5, 7, 8, 9, 10};
static constexpr int COL_PINS[COL_NUM] = {21, 17, 1, 6, 12, 13, 14, 15};

static constexpr uint32_t DEBOUNCE_US = 5000;


static constexpr uint16_t USB_VID = 0xA006;
static constexpr uint16_t USB_PID = 0x2E71;

static constexpr uint16_t LED_TARGET_FPS = 200;
static constexpr uint32_t LED_FRAME_INTERVAL_MS = 1000 / LED_TARGET_FPS;
static constexpr uint16_t MIDI_RX_QUEUE_SIZE = 128;
static constexpr uint16_t MIDI_TX_QUEUE_SIZE = 128;
