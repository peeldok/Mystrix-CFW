#include "cfy.h"

#include "config.h"
#include "led.h"
#include "matrix.h"
#include "types.h"

// Lookup replaces an O(10x10) scan for every CFY address.
static int8_t cfy_address_lut[128];
static bool cfy_lut_ready = false;

static void init_cfy_lut() {
    if (cfy_lut_ready) return;
    for (int i = 0; i < 128; ++i) {
        cfy_address_lut[i] = static_cast<int8_t>(nonlit_address_to_led(i));
    }
    cfy_lut_ready = true;
}

static inline void set_address_unlocked(uint8_t address, RGB color) {
    if (address >= 128) return;
    const int idx = cfy_address_lut[address];
    if (idx >= 0) led_set_pixel_unlocked(idx, color);
}

void cfy_handle(const uint8_t *data, size_t length) {
    size_t pos = 0;

    init_cfy_lut();
    led_begin_update();

    while (pos + 3 <= length) {
        const uint8_t raw_r = data[pos++];
        const uint8_t raw_g = data[pos++];
        const uint8_t raw_b = data[pos++];

        uint8_t count = 0;
        const bool compressed =
            (raw_r & 0x40) ||
            (raw_g & 0x40) ||
            (raw_b & 0x40);

        if (compressed) {
            count =
                ((raw_r & 0x40) ? 4 : 0) |
                ((raw_g & 0x40) ? 2 : 0) |
                ((raw_b & 0x40) ? 1 : 0);
        } else {
            if (pos >= length) break;
            count = data[pos++];
        }

        if (count == 0) continue;
        if (pos + count > length) break;

        const RGB color{
            static_cast<uint8_t>((raw_r & 0x3F) << 2),
            static_cast<uint8_t>((raw_g & 0x3F) << 2),
            static_cast<uint8_t>((raw_b & 0x3F) << 2)
        };

        for (uint8_t i = 0; i < count; ++i) {
            const uint8_t address = data[pos++];

            if (address == 0) {
                led_fill_unlocked(color);
                continue;
            }

            if (address >= 100 && address <= 109) {
                if (address == 100) {
                    for (uint8_t col = 1; col <= 8; ++col) {
                        set_address_unlocked(col, color);
                    }
                } else if (address == 109) {
                    for (uint8_t col = 1; col <= 8; ++col) {
                        set_address_unlocked(90 + col, color);
                    }
                } else {
                    const uint8_t row = address - 100;
                    for (uint8_t col = 1; col <= 8; ++col) {
                        set_address_unlocked(row * 10 + col, color);
                    }
                }
                continue;
            }

            if (address >= 110 && address <= 119) {
                if (address == 110) {
                    for (uint8_t row = 1; row <= 8; ++row) {
                        set_address_unlocked(row * 10, color);
                    }
                } else if (address == 119) {
                    for (uint8_t row = 1; row <= 8; ++row) {
                        set_address_unlocked(row * 10 + 9, color);
                    }
                } else {
                    const uint8_t col = address - 110;
                    for (uint8_t row = 1; row <= 8; ++row) {
                        set_address_unlocked(row * 10 + col, color);
                    }
                }
                continue;
            }

            if (address == 99) continue;
            set_address_unlocked(address, color);
        }
    }

    led_end_update(true);
}
