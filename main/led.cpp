#include "led.h"
#include "config.h"
#include "palette.h"

#include <atomic>
#include <cstdlib>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "led";
static constexpr uint32_t RMT_RESOLUTION_HZ = 10 * 1000 * 1000;
static constexpr size_t LED_DATA_BYTES = NUM_LEDS * 3;

static RGB framebuffer[NUM_LEDS]{};
static RGB boot_framebuffer[NUM_LEDS]{};
static uint8_t led_data[LED_DATA_BYTES]{};
static SemaphoreHandle_t framebuffer_mutex = nullptr;
static std::atomic<bool> led_dirty{false};
static std::atomic<bool> led_settings_ui_active{false};
static std::atomic<bool> led_boot_active{false};
static std::atomic<bool> led_settings_velocity_enabled{true};
static std::atomic<SettingsPage> settings_page_state{SettingsPage::Main};
static std::atomic<BrightnessTarget> settings_brightness_target_state{BrightnessTarget::Matrix};
static std::atomic<uint8_t> settings_matrix_brightness_state{255};
static std::atomic<uint8_t> settings_side_brightness_state{255};
static rmt_channel_handle_t rmt_channel = nullptr;
static rmt_encoder_handle_t rmt_encoder = nullptr;

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_handle_t bytes_encoder;
    rmt_encoder_handle_t copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} mystrix_led_encoder_t;

static size_t IRAM_ATTR encode_led(
    rmt_encoder_t *encoder,
    rmt_channel_handle_t channel,
    const void *primary_data,
    size_t data_size,
    rmt_encode_state_t *ret_state) {

    auto *led_encoder = reinterpret_cast<mystrix_led_encoder_t *>(encoder);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (led_encoder->state) {
        case 0:
            encoded_symbols += led_encoder->bytes_encoder->encode(
                led_encoder->bytes_encoder,
                channel,
                primary_data,
                data_size,
                &session_state
            );

            if (session_state & RMT_ENCODING_COMPLETE) {
                led_encoder->state = 1;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state = static_cast<rmt_encode_state_t>(state | RMT_ENCODING_MEM_FULL);
                goto out;
            }

        case 1:
            encoded_symbols += led_encoder->copy_encoder->encode(
                led_encoder->copy_encoder,
                channel,
                &led_encoder->reset_code,
                sizeof(led_encoder->reset_code),
                &session_state
            );

            if (session_state & RMT_ENCODING_COMPLETE) {
                led_encoder->state = 0;
                state = static_cast<rmt_encode_state_t>(state | RMT_ENCODING_COMPLETE);
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                state = static_cast<rmt_encode_state_t>(state | RMT_ENCODING_MEM_FULL);
            }
            break;
    }

out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t IRAM_ATTR delete_led_encoder(rmt_encoder_t *encoder) {
    auto *led_encoder = reinterpret_cast<mystrix_led_encoder_t *>(encoder);
    if (led_encoder->bytes_encoder) rmt_del_encoder(led_encoder->bytes_encoder);
    if (led_encoder->copy_encoder) rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t IRAM_ATTR reset_led_encoder(rmt_encoder_t *encoder) {
    auto *led_encoder = reinterpret_cast<mystrix_led_encoder_t *>(encoder);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = 0;
    return ESP_OK;
}

static esp_err_t create_led_encoder(rmt_encoder_handle_t *ret_encoder) {
    auto *encoder = static_cast<mystrix_led_encoder_t *>(calloc(1, sizeof(mystrix_led_encoder_t)));
    if (!encoder) return ESP_ERR_NO_MEM;

    encoder->base.encode = encode_led;
    encoder->base.del = delete_led_encoder;
    encoder->base.reset = reset_led_encoder;

    rmt_bytes_encoder_config_t bytes_cfg = {};
    bytes_cfg.bit0.level0 = 1;
    bytes_cfg.bit0.duration0 = 3;
    bytes_cfg.bit0.level1 = 0;
    bytes_cfg.bit0.duration1 = 9;
    bytes_cfg.bit1.level0 = 1;
    bytes_cfg.bit1.duration0 = 6;
    bytes_cfg.bit1.level1 = 0;
    bytes_cfg.bit1.duration1 = 6;
    bytes_cfg.flags.msb_first = 1;

    esp_err_t err = rmt_new_bytes_encoder(&bytes_cfg, &encoder->bytes_encoder);
    if (err != ESP_OK) {
        free(encoder);
        return err;
    }

    rmt_copy_encoder_config_t copy_cfg = {};
    err = rmt_new_copy_encoder(&copy_cfg, &encoder->copy_encoder);
    if (err != ESP_OK) {
        rmt_del_encoder(encoder->bytes_encoder);
        free(encoder);
        return err;
    }

    encoder->reset_code.level0 = 0;
    encoder->reset_code.duration0 = 1000;
    encoder->reset_code.level1 = 0;
    encoder->reset_code.duration1 = 0;
    encoder->state = 0;

    *ret_encoder = &encoder->base;
    return ESP_OK;
}

static inline uint8_t scale8(uint8_t value, uint8_t brightness) {
    return static_cast<uint8_t>((static_cast<uint16_t>(value) * brightness + 127) / 255);
}

static bool prepare_latest_frame() {
    if (xSemaphoreTake(framebuffer_mutex, 0) != pdTRUE) {
        return false;
    }

    if (!led_dirty.load(std::memory_order_acquire)) {
        xSemaphoreGive(framebuffer_mutex);
        return false;
    }

    const uint8_t master_brightness = g_brightness;
    if (led_boot_active.load(std::memory_order_acquire)) {
        for (int i = 0; i < NUM_LEDS; ++i) {
            const int offset = i * 3;
            led_data[offset + 0] = boot_framebuffer[i].g;
            led_data[offset + 1] = boot_framebuffer[i].r;
            led_data[offset + 2] = boot_framebuffer[i].b;
        }
    } else if (led_settings_ui_active.load(std::memory_order_acquire)) {
        for (size_t i = 0; i < LED_DATA_BYTES; ++i) led_data[i] = 0;

        auto set_ui_pixel = [](int index, RGB color, uint8_t brightness) {
            if (index < 0 || index >= NUM_LEDS) return;
            const int offset = index * 3;
            led_data[offset + 0] = scale8(color.g, brightness);
            led_data[offset + 1] = scale8(color.r, brightness);
            led_data[offset + 2] = scale8(color.b, brightness);
        };

        const SettingsPage page = settings_page_state.load(std::memory_order_acquire);
        if (page == SettingsPage::Main) {
            const bool enabled = led_settings_velocity_enabled.load(std::memory_order_acquire);
            const RGB yellow = enabled ? RGB{255, 180, 0} : RGB{24, 17, 0};
            set_ui_pixel(0, yellow, 255);

            const RGB brightness_entry = {255, 255, 255};
            set_ui_pixel(27, brightness_entry, 255);
            set_ui_pixel(28, brightness_entry, 255);
            set_ui_pixel(35, brightness_entry, 255);
            set_ui_pixel(36, brightness_entry, 255);

            const RGB tinyuf2_button = {255, 0, 0};
            set_ui_pixel(63, tinyuf2_button, 255);
        } else {
            const BrightnessTarget target = settings_brightness_target_state.load(std::memory_order_acquire);
            const uint8_t matrix_brightness = settings_matrix_brightness_state.load(std::memory_order_acquire);
            const uint8_t side_brightness = settings_side_brightness_state.load(std::memory_order_acquire);
            const uint8_t selected_brightness = target == BrightnessTarget::Matrix
                ? matrix_brightness
                : side_brightness;

            uint8_t selected_step = static_cast<uint8_t>(
                (static_cast<uint16_t>(selected_brightness) * 16U + 127U) / 255U
            );
            if (selected_step < 1) selected_step = 1;
            if (selected_step > 16) selected_step = 16;

            const RGB matrix_selector = {0, 255, 0};
    const RGB side_selector = {0, 96, 255};
            set_ui_pixel(40, matrix_selector, target == BrightnessTarget::Matrix ? 255 : 32);
            set_ui_pixel(41, side_selector, target == BrightnessTarget::Side ? 255 : 32);

            const RGB level = {255, 255, 255};
            for (uint8_t step = 1; step <= 16; ++step) {
                const int index = step <= 8 ? 48 + (step - 1) : 56 + (step - 9);
                const uint8_t brightness = step <= selected_step ? selected_brightness : 4;
                set_ui_pixel(index, level, brightness);
            }
        }
    } else {
        const uint8_t matrix_brightness = settings_matrix_brightness_state.load(std::memory_order_acquire);
        const uint8_t side_brightness = settings_side_brightness_state.load(std::memory_order_acquire);

        for (int i = 0; i < NUM_LEDS; ++i) {
            const int offset = i * 3;
            const uint8_t region_brightness = i < 64 ? matrix_brightness : side_brightness;
            const uint8_t effective_brightness = scale8(master_brightness, region_brightness);
            led_data[offset + 0] = scale8(framebuffer[i].g, effective_brightness);
            led_data[offset + 1] = scale8(framebuffer[i].r, effective_brightness);
            led_data[offset + 2] = scale8(framebuffer[i].b, effective_brightness);
        }
    }

    led_dirty.store(false, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
    return true;
}

static void led_task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LED_FRAME_INTERVAL_MS);

    const rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 1,
        },
    };

    for (;;) {
        vTaskDelayUntil(&last_wake, period);

        if (!led_dirty.load(std::memory_order_acquire)) {
            continue;
        }


        if (rmt_tx_wait_all_done(rmt_channel, 0) != ESP_OK) {
            continue;
        }

        if (!prepare_latest_frame()) {
            continue;
        }

        const esp_err_t err = rmt_transmit(
            rmt_channel,
            rmt_encoder,
            led_data,
            sizeof(led_data),
            &tx_cfg
        );

        if (err != ESP_OK) {

            xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
            led_dirty.store(true, std::memory_order_release);
            xSemaphoreGive(framebuffer_mutex);
        }
    }
}

void led_init() {
    framebuffer_mutex = xSemaphoreCreateMutex();
    configASSERT(framebuffer_mutex);

    rmt_tx_channel_config_t channel_cfg = {};
    channel_cfg.gpio_num = static_cast<gpio_num_t>(LED_GPIO);
    channel_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    channel_cfg.resolution_hz = RMT_RESOLUTION_HZ;
    channel_cfg.mem_block_symbols = 256;
    channel_cfg.trans_queue_depth = 1;
    channel_cfg.intr_priority = 0;
    channel_cfg.flags.invert_out = 0;
    channel_cfg.flags.with_dma = 1;
    channel_cfg.flags.io_loop_back = 0;
    channel_cfg.flags.io_od_mode = 0;

    esp_err_t err = rmt_new_tx_channel(&channel_cfg, &rmt_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
        return;
    }

    err = create_led_encoder(&rmt_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "create_led_encoder failed: %s", esp_err_to_name(err));
        return;
    }

    err = rmt_enable(rmt_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(err));
        return;
    }

    xTaskCreatePinnedToCore(
        led_task,
        "led_200fps",
        4096,
        nullptr,
        5,
        nullptr,
        0
    );

    led_fill({0, 0, 0});
    led_mark_dirty();
    ESP_LOGI(TAG, "RMT DMA LED driver started at target %u FPS", LED_TARGET_FPS);
}

void led_set_pixel(int index, RGB color) {
    if (index < 0 || index >= NUM_LEDS) return;
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    framebuffer[index] = color;
    led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
}

void led_fill(RGB color) {
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    for (auto &pixel : framebuffer) pixel = color;
    led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
}

void led_set_brightness(uint8_t brightness) {
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    g_brightness = brightness;
    led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
}

void led_mark_dirty() {
    led_dirty.store(true, std::memory_order_release);
}

void led_begin_update() {
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
}

void led_set_pixel_unlocked(int index, RGB color) {
    if (index < 0 || index >= NUM_LEDS) return;
    framebuffer[index] = color;
}

void led_fill_unlocked(RGB color) {
    for (auto &pixel : framebuffer) pixel = color;
}

void led_end_update(bool notify) {
    if (notify) led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
}

void led_set_settings_ui(
    bool active,
    bool velocity_enabled,
    SettingsPage page,
    BrightnessTarget target,
    uint8_t matrix_brightness,
    uint8_t side_brightness
) {
    led_settings_ui_active.store(active, std::memory_order_release);
    led_settings_velocity_enabled.store(velocity_enabled, std::memory_order_release);
    settings_page_state.store(page, std::memory_order_release);
    settings_brightness_target_state.store(target, std::memory_order_release);
    settings_matrix_brightness_state.store(matrix_brightness, std::memory_order_release);
    settings_side_brightness_state.store(side_brightness, std::memory_order_release);
    led_dirty.store(true, std::memory_order_release);
}

void led_boot_begin() {
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    for (auto &pixel : boot_framebuffer) pixel = RGB{0, 0, 0};
    led_boot_active.store(true, std::memory_order_release);
    led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
}

void led_boot_end() {
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    led_boot_active.store(false, std::memory_order_release);
    led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
}

void led_boot_set_pixel(int index, RGB color) {
    if (index < 0 || index >= NUM_LEDS) return;
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    boot_framebuffer[index] = color;
    xSemaphoreGive(framebuffer_mutex);
}

void led_boot_fill(RGB color) {
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    for (auto &pixel : boot_framebuffer) pixel = color;
    xSemaphoreGive(framebuffer_mutex);
}

void led_boot_mark_dirty() {
    led_dirty.store(true, std::memory_order_release);
}
