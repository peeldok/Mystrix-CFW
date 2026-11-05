#pragma once

// Asynchronous copy of ESP-IDF logs to the extra USB CDC ACM debug interface.
void cdc_debug_init();

#include "tinyusb_cdc_acm.h"

// TinyUSB task RX callback: accept the one-byte B benchmark command.
void cdc_debug_rx_callback(int itf, cdcacm_event_t *event);
