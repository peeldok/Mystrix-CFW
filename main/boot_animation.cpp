#include "boot_animation.h"
#include "led.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr int GRID_SIZE = 8;
static constexpr uint32_t PHASE1_SECTION_MS = 80;
static constexpr uint32_t PHASE2_START_OFFSET_MS = 150;
static constexpr float BOOT_BRIGHTNESS = 1.0f;

static int xy_to_led(int x, int y) {
    if (x >= 0 && x < 8 && y >= 0 && y < 8) return x + y * 8;
    if (x == 8 && y >= 0 && y < 8) return 64 + (7 - y);
    if (y == -1 && x >= 0 && x < 8) return 72 + (7 - x);
    if (x == -1 && y >= 0 && y < 8) return 80 + y;
    if (y == 8 && x >= 0 && x < 8) return 88 + x;
    return -1;
}

static RGB hsv_to_rgb(float hue, float saturation, float value) {
    hue = hue - std::floor(hue);
    saturation = std::clamp(saturation, 0.0f, 1.0f);
    value = std::clamp(value, 0.0f, 1.0f);

    const float h = hue * 6.0f;
    const int region = static_cast<int>(std::floor(h)) % 6;
    const float f = h - std::floor(h);
    const float p = value * (1.0f - saturation);
    const float q = value * (1.0f - saturation * f);
    const float t = value * (1.0f - saturation * (1.0f - f));

    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;

    switch (region) {
        case 0: r = value; g = t; b = p; break;
        case 1: r = q; g = value; b = p; break;
        case 2: r = p; g = value; b = t; break;
        case 3: r = p; g = q; b = value; break;
        case 4: r = t; g = p; b = value; break;
        default: r = value; g = p; b = q; break;
    }

    return RGB{
        static_cast<uint8_t>(std::lround(r * 255.0f)),
        static_cast<uint8_t>(std::lround(g * 255.0f)),
        static_cast<uint8_t>(std::lround(b * 255.0f))
    };
}

static RGB phase2_color(int32_t time_ms, float hue) {
    float saturation;
    float brightness;

    if (time_ms < 0) {
        saturation = 0.0f;
    } else if (time_ms < 400) {
        saturation = static_cast<float>(time_ms) / 400.0f;
    } else {
        saturation = 1.0f;
    }

    if (time_ms < -100) {
        brightness = 0.0f;
    } else if (time_ms < 0) {
        brightness = (static_cast<float>(time_ms + 100) / 100.0f) * BOOT_BRIGHTNESS;
    } else if (time_ms < 300) {
        brightness = BOOT_BRIGHTNESS;
    } else if (time_ms < 500) {
        brightness = (1.0f - static_cast<float>(time_ms - 300) / 200.0f) * BOOT_BRIGHTNESS;
    } else {
        brightness = 0.0f;
    }

    return hsv_to_rgb(hue, saturation, brightness);
}

static void frame_clear() {
    led_boot_fill({0, 0, 0});
}

static void frame_set_xy(int x, int y, RGB color) {
    const int index = xy_to_led(x, y);
    if (index >= 0) led_boot_set_pixel(index, color);
}

static void quad_set_color(int x_offset, int y_offset, RGB color1, RGB color2) {
    const int ox = 3;
    const int oy = 3;

    frame_set_xy(ox + 1 + x_offset, oy + 1 + y_offset, color2);
    frame_set_xy(ox - x_offset, oy + 1 + y_offset, color1);
    frame_set_xy(ox - x_offset, oy - y_offset, color2);
    frame_set_xy(ox + 1 + x_offset, oy - y_offset, color1);
}

static void run_phase1() {
    const int ox = 3;
    const int oy = 3;

    frame_clear();
    led_boot_mark_dirty();

    for (int counter = 0; counter <= 6; ++counter) {
        const TickType_t start = xTaskGetTickCount();

        for (;;) {
            const uint32_t elapsed = static_cast<uint32_t>(
                (xTaskGetTickCount() - start) * portTICK_PERIOD_MS
            );

            const uint8_t brightness = static_cast<uint8_t>(
                std::min<uint32_t>(255, (elapsed * 255U) / PHASE1_SECTION_MS)
            );
            const RGB color{brightness, brightness, brightness};

            if (counter <= 3) {
                const int line_x = ox - 1;
                const int line_y = oy - 1 + counter;
                for (int i = 0; i < counter + 1; ++i) {
                    frame_set_xy(line_x + i, line_y - i, color);
                }
            } else {
                const int line_x = ox + counter - 4;
                const int line_y = oy + 2;
                for (int i = 0; i < 3 - (counter - 4); ++i) {
                    frame_set_xy(line_x + i, line_y - i, color);
                }
            }

            led_boot_mark_dirty();

            if (elapsed >= PHASE1_SECTION_MS) break;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}

static void run_phase2() {
    const float hue1 = 0.5f;
    const float hue2 = 0.833f;
    const int quad_size = GRID_SIZE / 2 + 1;
    const uint32_t total_time = static_cast<uint32_t>(
        (quad_size - 2) * PHASE2_START_OFFSET_MS + 800
    );

    const TickType_t start = xTaskGetTickCount();

    for (;;) {
        const uint32_t elapsed = static_cast<uint32_t>(
            (xTaskGetTickCount() - start) * portTICK_PERIOD_MS
        );

        frame_clear();

        for (int r = 0; r < quad_size; ++r) {
            const int32_t local_time =
                static_cast<int32_t>(elapsed) - (r - 1) * static_cast<int32_t>(PHASE2_START_OFFSET_MS);

            const RGB color1 = phase2_color(local_time, hue1);
            const RGB color2 = phase2_color(local_time, hue2);
            quad_set_color(r, r, color1, color2);

            if (r > 0) {
                const int32_t half_time =
                    local_time + static_cast<int32_t>(PHASE2_START_OFFSET_MS / 2);
                const RGB half1 = phase2_color(half_time, hue1);
                const RGB half2 = phase2_color(half_time, hue2);
                quad_set_color(r - 1, r, half1, half2);
                quad_set_color(r, r - 1, half1, half2);
            }

            if (r > 3) {
                const int32_t outer_time =
                    local_time + static_cast<int32_t>(PHASE2_START_OFFSET_MS * 3 / 2);
                const RGB outer1 = phase2_color(outer_time, hue1);
                const RGB outer2 = phase2_color(outer_time, hue2);
                quad_set_color(r - 2, r, outer1, outer2);
                quad_set_color(r, r - 2, outer1, outer2);
            }
        }

        led_boot_mark_dirty();

        if (elapsed > total_time) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void boot_task(void *) {
    vTaskDelay(pdMS_TO_TICKS(100));
    led_boot_begin();
    run_phase1();
    run_phase2();
    frame_clear();
    led_boot_mark_dirty();
    vTaskDelay(pdMS_TO_TICKS(20));
    led_boot_end();
    vTaskDelete(nullptr);
}

void boot_animation_start() {
    xTaskCreatePinnedToCore(
        boot_task,
        "boot_animation",
        4096,
        nullptr,
        4,
        nullptr,
        0
    );
}
