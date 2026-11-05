#include "midi.h"

#include <atomic>
#include <cstring>

#include "cfy.h"
#include "config.h"
#include "cdc_debug.h"
#include "fsr.h"
#include "led.h"
#include "matrix.h"
#include "palette.h"
#include "types.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

static const char *TAG = "midi";

static constexpr uint8_t MIDI_PORT_COUNT = 2;

static constexpr uint8_t MIDI1_EP_OUT = 0x01;
static constexpr uint8_t MIDI1_EP_IN  = 0x81;
static constexpr uint8_t MIDI2_EP_OUT = 0x02;
static constexpr uint8_t MIDI2_EP_IN  = 0x82;
static constexpr uint8_t CDC_EP_NOTIF = 0x83;
static constexpr uint8_t CDC_EP_OUT = 0x04;
static constexpr uint8_t CDC_EP_IN = 0x84;

enum {
    ITF_NUM_MIDI1 = 0,
    ITF_NUM_MIDI1_STREAMING,
    ITF_NUM_MIDI2,
    ITF_NUM_MIDI2_STREAMING,
    ITF_NUM_CDC_CONTROL,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

static constexpr uint16_t USB_CONFIG_TOTAL_LEN =
    TUD_CONFIG_DESC_LEN +
    TUD_MIDI_DESC_LEN +
    TUD_MIDI_DESC_LEN +
    TUD_CDC_DESC_LEN;

static const tusb_desc_device_t usb_device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0102,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

static const uint8_t usb_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(
        1,
        ITF_NUM_TOTAL,
        0,
        USB_CONFIG_TOTAL_LEN,
        TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
        100
    ),

    TUD_MIDI_DESCRIPTOR(
        ITF_NUM_MIDI1,
        4,
        MIDI1_EP_OUT,
        MIDI1_EP_IN,
        64
    ),

    TUD_MIDI_DESCRIPTOR(
        ITF_NUM_MIDI2,
        5,
        MIDI2_EP_OUT,
        MIDI2_EP_IN,
        64
    ),

    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC_CONTROL,
        6,
        CDC_EP_NOTIF,
        8,
        CDC_EP_OUT,
        CDC_EP_IN,
        64
    ),
};

static const char usb_langid[] = {0x09, 0x04};
static const char *usb_string_descriptors[] = {
    usb_langid,
    "PeelDok",
    "Mystrix",
    "Mystrix",
    "Mystrix (CFW)",
    "Mystrix (CFW)",
    "Mystrix Debug CDC",
};

struct UsbMidiPacket {
    uint8_t port;
    uint8_t data[4];
};

static QueueHandle_t midi_rx_queue = nullptr;
static QueueHandle_t button_queue = nullptr;
static std::atomic<uint32_t> dropped_rx_packets{0};
static std::atomic<uint32_t> received_rx_packets{0};
static std::atomic<uint32_t> rx_queue_peak{0};
static uint32_t dropped_tx_events = 0;
static constexpr uint32_t MIDI_PARSE_BATCH_LIMIT = 192;
static constexpr int64_t MIDI_PARSE_BUDGET_US = 750;

// Only display the most recent state of a pad within one MIDI parse batch.
// Note and SysEx ordering is preserved by flushing pending notes before SysEx.
static RGB staged_note_colors[NUM_LEDS]{};
static uint8_t staged_note_mask[NUM_LEDS]{};
static bool staged_note_dirty = false;

static void flush_note_batch() {
    if (!staged_note_dirty) return;
    led_begin_update();
    for (int i = 0; i < NUM_LEDS; ++i) {
        if (!staged_note_mask[i]) continue;
        led_set_pixel_unlocked(i, staged_note_colors[i]);
        staged_note_mask[i] = 0;
    }
    led_end_update(true);
    staged_note_dirty = false;
}

static constexpr size_t SYSEX_BUFFER_SIZE = 1024;

struct SysExState {
    uint8_t buffer[SYSEX_BUFFER_SIZE];
    size_t len;
    bool active;
};

static SysExState sysex_state[MIDI_PORT_COUNT]{};

static void enqueue_rx_drop_oldest(const UsbMidiPacket &packet) {
    if (!midi_rx_queue) return;

    if (uxQueueSpacesAvailable(midi_rx_queue) == 0) {
        UsbMidiPacket discarded{};
        xQueueReceive(midi_rx_queue, &discarded, 0);
        dropped_rx_packets.fetch_add(1, std::memory_order_relaxed);
    }
    if (xQueueSend(midi_rx_queue, &packet, 0) == pdTRUE) {
        received_rx_packets.fetch_add(1, std::memory_order_relaxed);
        uint32_t size = static_cast<uint32_t>(uxQueueMessagesWaiting(midi_rx_queue));
        uint32_t peak = rx_queue_peak.load(std::memory_order_relaxed);
        while (size > peak && !rx_queue_peak.compare_exchange_weak(
            peak, size, std::memory_order_relaxed)) {}
    }
}

extern "C" void tud_midi_rx_cb(uint8_t itf) {
    if (itf >= MIDI_PORT_COUNT) return;

    UsbMidiPacket packet{};
    packet.port = itf;

    while (tud_midi_n_packet_read(itf, packet.data)) {
        enqueue_rx_drop_oldest(packet);
    }
}

static void midi_send_3_to_port(uint8_t port, uint8_t status, uint8_t d1, uint8_t d2) {
    if (port >= MIDI_PORT_COUNT) return;

    const uint8_t msg[3] = {status, d1, d2};
    tud_midi_n_stream_write(port, 0, msg, sizeof(msg));
}

static void midi_send_3_all_ports(uint8_t status, uint8_t d1, uint8_t d2) {
    for (uint8_t port = 0; port < MIDI_PORT_COUNT; ++port) {
        midi_send_3_to_port(port, status, d1, d2);
    }
}

void midi_queue_button(uint8_t note, bool pressed, uint8_t velocity) {
    if (!button_queue) return;

    const ButtonEvent ev{note, pressed, velocity};
    if (uxQueueSpacesAvailable(button_queue) == 0) {
        ButtonEvent discarded{};
        xQueueReceive(button_queue, &discarded, 0);
        ++dropped_tx_events;
    }
    xQueueSend(button_queue, &ev, 0);
}

static void handle_note(uint8_t status, uint8_t note, uint8_t velocity) {
    const uint8_t type = status & 0xF0;
    const uint8_t channel = status & 0x0F;

    const bool is_note_on = type == 0x90 && velocity != 0;
    const bool is_note_off = type == 0x80 || (type == 0x90 && velocity == 0);
    if (!is_note_on && !is_note_off) return;

    const int idx = drum_pitch_to_led(note);
    if (idx < 0 || idx >= NUM_LEDS) return;

    staged_note_colors[idx] = is_note_off
        ? RGB{0, 0, 0}
        : palette_color(channel, velocity);
    staged_note_mask[idx] = 1;
    staged_note_dirty = true;
}

static void send_identity_reply(uint8_t port) {
    if (port >= MIDI_PORT_COUNT) return;

    static const uint8_t response[] = {
        0xF0, 0x7E, 0x00, 0x06, 0x02,
        0x00, 0x20, 0x29, 0x51, 0x00,
        0x00, 0x00, 0x00, 0x63, 0x66, 0x79, 0xF7
    };

    tud_midi_n_stream_write(port, 0, response, sizeof(response));
}

static void send_web_identity_reply(uint8_t port, uint8_t token) {
    if (port >= MIDI_PORT_COUNT) return;

    const uint8_t response[] = {
        0xF0, 0x7D, 0x4D, 0x59, 0x58, 0x02,
        static_cast<uint8_t>(token & 0x7F),
        0x01,
        0x01, 0x00, 0x01,
        0xF7
    };

    tud_midi_n_stream_write(port, 0, response, sizeof(response));
}


static void send_velocity_config_reply(uint8_t port) {
    if (port >= MIDI_PORT_COUNT) return;

    const uint32_t min_slope = fsr_min_slope();
    const uint32_t max_slope = fsr_max_slope();

    const uint8_t response[] = {
        0xF0, 0x7D, 0x4D, 0x59, 0x58, 0x04,
        static_cast<uint8_t>(fsr_min_velocity() & 0x7F),
        static_cast<uint8_t>(fsr_max_velocity() & 0x7F),
        static_cast<uint8_t>((min_slope >> 7) & 0x7F),
        static_cast<uint8_t>(min_slope & 0x7F),
        static_cast<uint8_t>((max_slope >> 7) & 0x7F),
        static_cast<uint8_t>(max_slope & 0x7F),
        0xF7
    };

    tud_midi_n_stream_write(port, 0, response, sizeof(response));
}

static void handle_vendor_sysex(uint8_t port, const uint8_t *data, size_t len) {
    if (len < 2 || data[0] != 0xF0 || data[len - 1] != 0xF7) return;

    if (len >= 3 && data[1] == 0x5F) {
        cfy_handle(&data[2], len - 3);
        return;
    }

    if (len >= 6 && data[1] == 0x7E && data[3] == 0x06 && data[4] == 0x01) {
        send_identity_reply(port);
        return;
    }

    if (len == 8 &&
        data[1] == 0x7D &&
        data[2] == 0x4D &&
        data[3] == 0x59 &&
        data[4] == 0x58 &&
        data[5] == 0x01) {
        send_web_identity_reply(port, data[6]);
        return;
    }

    if (len == 7 &&
        data[1] == 0x7D &&
        data[2] == 0x4D &&
        data[3] == 0x59 &&
        data[4] == 0x58 &&
        data[5] == 0x03) {
        send_velocity_config_reply(port);
        return;
    }

    if (len == 13 &&
        data[1] == 0x7D &&
        data[2] == 0x4D &&
        data[3] == 0x59 &&
        data[4] == 0x58 &&
        data[5] == 0x05) {
        const uint8_t min_velocity = data[6] & 0x7F;
        const uint8_t max_velocity = data[7] & 0x7F;
        const uint32_t min_slope = (static_cast<uint32_t>(data[8] & 0x7F) << 7) | (data[9] & 0x7F);
        const uint32_t max_slope = (static_cast<uint32_t>(data[10] & 0x7F) << 7) | (data[11] & 0x7F);
        fsr_set_velocity_config(min_velocity, max_velocity, min_slope, max_slope);
        send_velocity_config_reply(port);
        return;
    }

    if (len < 4 || data[1] != 0x7D) return;

    const uint8_t p_idx = data[2];
    if (p_idx == 0x0F) {
        if (len >= 7) {
            const uint16_t value = (static_cast<uint16_t>(data[4]) << 7) | data[5];
            led_set_brightness(static_cast<uint8_t>(value > 255 ? 255 : value));
        }
        return;
    }

    if (p_idx >= 3 || len < 261) return;
    const uint8_t component = data[3];
    if (component >= 3) return;

    uint8_t decoded[128];
    for (int i = 0; i < 128; ++i) {
        decoded[i] = static_cast<uint8_t>(
            (static_cast<uint16_t>(data[4 + i * 2]) << 7) |
            data[5 + i * 2]
        );
    }
    palette_save_web(p_idx, component, decoded);
}

static void sysex_push(uint8_t port, uint8_t byte) {
    if (port >= MIDI_PORT_COUNT) return;

    SysExState &state = sysex_state[port];

    if (byte == 0xF0) {
        state.len = 0;
        state.active = true;
    }

    if (!state.active) return;

    if (state.len >= sizeof(state.buffer)) {
        state.len = 0;
        state.active = false;
        return;
    }

    state.buffer[state.len++] = byte;

    if (byte == 0xF7) {
        handle_vendor_sysex(port, state.buffer, state.len);
        state.len = 0;
        state.active = false;
    }
}

static uint8_t cin_data_length(uint8_t cin) {
    switch (cin) {
        case 0x2: return 2;
        case 0x3: return 3;
        case 0x4: return 3;
        case 0x5: return 1;
        case 0x6: return 2;
        case 0x7: return 3;
        case 0x8: return 3;
        case 0x9: return 3;
        case 0xA: return 3;
        case 0xB: return 3;
        case 0xC: return 2;
        case 0xD: return 2;
        case 0xE: return 3;
        case 0xF: return 1;
        default: return 0;
    }
}

static void process_usb_packet(uint8_t port, const uint8_t packet[4]) {
    if (port >= MIDI_PORT_COUNT) return;

    const uint8_t cin = packet[0] & 0x0F;
    const uint8_t len = cin_data_length(cin);
    if (len == 0) return;

    SysExState &state = sysex_state[port];

    if (state.active ||
        packet[1] == 0xF0 ||
        cin == 0x4 ||
        cin == 0x5 ||
        cin == 0x6 ||
        cin == 0x7) {
        flush_note_batch(); // An entire SysEx must remain ordered after preceding Notes.
        for (uint8_t i = 0; i < len; ++i) {
            sysex_push(port, packet[1 + i]);
        }
        return;
    }

    const uint8_t status = packet[1];
    if ((status & 0xF0) == 0x80 || (status & 0xF0) == 0x90) {
        handle_note(status, packet[2], packet[3]);
    }
}

static void midi_parser_task(void *) {
    UsbMidiPacket packet{};
    uint32_t parsed_in_window = 0;
    int64_t stats_start_us = esp_timer_get_time();
    uint32_t last_drop_count = 0;

    for (;;) {
        if (xQueueReceive(midi_rx_queue, &packet, pdMS_TO_TICKS(25)) == pdTRUE) {
            const int64_t batch_start_us = esp_timer_get_time();
            uint32_t processed = 0;
            do {
                process_usb_packet(packet.port, packet.data);
                ++processed;
                // Limit processing time to avoid monopolizing core 1.
                if (processed >= MIDI_PARSE_BATCH_LIMIT ||
                    esp_timer_get_time() - batch_start_us >= MIDI_PARSE_BUDGET_US) {
                    break;
                }
            } while (xQueueReceive(midi_rx_queue, &packet, 0) == pdTRUE);
            flush_note_batch();
            parsed_in_window += processed;
        }

        const int64_t now_us = esp_timer_get_time();
        const int64_t elapsed_us = now_us - stats_start_us;
        if (elapsed_us >= 1000000) {
            const uint32_t receive_count = received_rx_packets.exchange(0, std::memory_order_relaxed);
            const uint32_t drop_count = dropped_rx_packets.load(std::memory_order_relaxed);
            const uint32_t peak = rx_queue_peak.exchange(0, std::memory_order_relaxed);
            ESP_LOGI(TAG, "PERF MIDI RX=%lu/s parsed=%lu/s peak=%lu/%u drops=%lu",
                static_cast<unsigned long>((static_cast<uint64_t>(receive_count) * 1000000ULL) / elapsed_us),
                static_cast<unsigned long>((static_cast<uint64_t>(parsed_in_window) * 1000000ULL) / elapsed_us),
                static_cast<unsigned long>(peak), MIDI_RX_QUEUE_SIZE,
                static_cast<unsigned long>(drop_count - last_drop_count));
            last_drop_count = drop_count;
            parsed_in_window = 0;
            stats_start_us = now_us;
        }
    }
}

static void midi_tx_task(void *) {
    ButtonEvent ev{};
    for (;;) {
        if (xQueueReceive(button_queue, &ev, portMAX_DELAY) == pdTRUE) {
            if (ev.pressed) {
                midi_send_3_all_ports(0x90, ev.note, ev.velocity == 0 ? 1 : ev.velocity);
            } else {
                midi_send_3_all_ports(0x80, ev.note, 0);
            }
        }
    }
}

void midi_init() {
    midi_rx_queue = xQueueCreate(MIDI_RX_QUEUE_SIZE, sizeof(UsbMidiPacket));
    button_queue = xQueueCreate(MIDI_TX_QUEUE_SIZE, sizeof(ButtonEvent));
    configASSERT(midi_rx_queue);
    configASSERT(button_queue);

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG();
    tusb_cfg.phy.self_powered = false;
    tusb_cfg.phy.vbus_monitor_io = -1;
    tusb_cfg.descriptor.device = &usb_device_descriptor;
    tusb_cfg.descriptor.string = usb_string_descriptors;
    tusb_cfg.descriptor.string_count = sizeof(usb_string_descriptors) / sizeof(usb_string_descriptors[0]);
    tusb_cfg.descriptor.full_speed_config = usb_configuration_descriptor;

    const esp_err_t usb_err = tinyusb_driver_install(&tusb_cfg);
    if (usb_err != ESP_OK) {
        ESP_LOGE(TAG, "tinyusb_driver_install failed: %s", esp_err_to_name(usb_err));
        return;
    }

    const tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = cdc_debug_rx_callback,
        .callback_rx_wanted_char = nullptr,
        .callback_line_state_changed = nullptr,
        .callback_line_coding_changed = nullptr,
    };
    const esp_err_t cdc_err = tinyusb_cdcacm_init(&acm_cfg);
    if (cdc_err != ESP_OK) {
        ESP_LOGE(TAG, "tinyusb_cdcacm_init failed: %s", esp_err_to_name(cdc_err));
    } else {
        cdc_debug_init();
    }

    xTaskCreatePinnedToCore(
        midi_parser_task,
        "midi_parser",
        6144,
        nullptr,
        3,
        nullptr,
        1
    );

    xTaskCreatePinnedToCore(
        midi_tx_task,
        "midi_tx",
        3072,
        nullptr,
        3,
        nullptr,
        1
    );

    ESP_LOGI(TAG, "Dual MIDI + CDC initialized: MIDI 1/2, Debug COM, VID=0xA006, PID=0x2E71");
}
