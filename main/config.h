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

static constexpr uint32_t LED_FRAME_INTERVAL_US = 3000;  // Exact 3 ms periodic timer
static constexpr uint16_t LED_TARGET_FPS = 1000000UL / LED_FRAME_INTERVAL_US;  // 333 FPS nominal
// USB FS bulk endpoint max packet size stays at 64 B. These are software
// FIFO sizes for each of the TWO MIDI interfaces, not USB endpoint MPS.
static constexpr uint16_t MIDI_RX_QUEUE_SIZE = 2048;
static constexpr uint16_t MIDI_TX_QUEUE_SIZE = 512;
static constexpr uint16_t MIDI_RX_FIFO_BYTES = 4096;
static constexpr uint16_t MIDI_TX_FIFO_BYTES = 1024;

// Keep the performance-critical tasks on distinct, explicit cores.
// CPU0: RMT LED updates, FSR scan, boot/status UI, low-priority diagnostics.
// CPU1: TinyUSB and MIDI RX/TX; TinyUSB must preempt the MIDI parser.
static constexpr int CPU_LED = 0;
static constexpr int CPU_FSR = 0;
static constexpr int CPU_BOOT = 0;
static constexpr int CPU_STATS = 0;
static constexpr int CPU_BENCHMARK = 0;
static constexpr int CPU_USB = 1;
static constexpr int CPU_MIDI_PARSER = 1;
static constexpr int CPU_MIDI_TX = 1;
static constexpr int CPU_CDC = 1;

static constexpr int PRIO_LED = 7;
static constexpr int PRIO_FSR = 4;
static constexpr int PRIO_BOOT = 3;
static constexpr int PRIO_BENCHMARK = 2;
static constexpr int PRIO_STATS = 1;
static constexpr int PRIO_USB = 8;
static constexpr int PRIO_MIDI_PARSER = 6;
static constexpr int PRIO_MIDI_TX = 4;
static constexpr int PRIO_CDC = 1;

