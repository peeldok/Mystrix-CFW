#include "led.h"
#include "config.h"
#include "matrix.h"
#include "palette.h"

#include <atomic>
#include <cstdlib>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "led";
static constexpr uint32_t RMT_RESOLUTION_HZ = 10 * 1000 * 1000;
static constexpr size_t LED_DATA_BYTES = NUM_LEDS * 3;

static RGB framebuffer[NUM_LEDS]{};
static RGB boot_framebuffer[NUM_LEDS]{};
// RMT uses a DMA payload pointer until the transfer-complete callback.
DRAM_ATTR static uint8_t led_data[2][LED_DATA_BYTES]{};
static int8_t logical_to_physical_lut[NUM_LEDS]{};
static RotationMode lut_rotation = RotationMode::Deg0;
static bool lut_ready = false;
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
static TaskHandle_t led_task_handle = nullptr;
static esp_timer_handle_t led_pacing_timer = nullptr;
static std::atomic<bool> led_rmt_busy{false};
static std::atomic<uint32_t> led_completed_frames{0};

static void led_timer_callback(void *arg) {
    xTaskNotifyGive(static_cast<TaskHandle_t>(arg));
}

// Only wake the LED task from interrupt context. Never render pixels in ISR.
static bool IRAM_ATTR led_on_rmt_done(
    rmt_channel_handle_t channel,
    const rmt_tx_done_event_data_t *edata,
    void *user_data) {
    (void)channel;
    (void)edata;
    (void)user_data;
    led_rmt_busy.store(false, std::memory_order_release);
    led_completed_frames.fetch_add(1, std::memory_order_relaxed);
    BaseType_t wake = pdFALSE;
    if (led_task_handle) {
        vTaskNotifyGiveFromISR(led_task_handle, &wake);
    }
    return wake == pdTRUE;
}

static inline void notify_led_task() {
    if (led_task_handle) {
        xTaskNotifyGive(led_task_handle);
    }
}

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

static bool prepare_latest_frame(uint8_t *frame_output) {
    if (xSemaphoreTake(framebuffer_mutex, 0) != pdTRUE) {
        return false;
    }

    // Consume dirty before rendering. Concurrent changes can mark dirty again,
    // avoiding a lost update while the output frame is being prepared.
    if (!led_dirty.exchange(false, std::memory_order_acq_rel)) {
        xSemaphoreGive(framebuffer_mutex);
        return false;
    }

    const uint8_t master_brightness = g_brightness;
    if (led_boot_active.load(std::memory_order_acquire)) {
        for (size_t i = 0; i < LED_DATA_BYTES; ++i) frame_output[i] = 0;
        for (int i = 0; i < NUM_LEDS; ++i) {
            const int physical_index = matrix_logical_led_to_physical(i);
            if (physical_index < 0 || physical_index >= NUM_LEDS) continue;
            const int offset = physical_index * 3;
            frame_output[offset + 0] = boot_framebuffer[i].g;
            frame_output[offset + 1] = boot_framebuffer[i].r;
            frame_output[offset + 2] = boot_framebuffer[i].b;
        }
    } else if (led_settings_ui_active.load(std::memory_order_acquire)) {
        for (size_t i = 0; i < LED_DATA_BYTES; ++i) frame_output[i] = 0;

        auto set_ui_pixel = [](int index, RGB color, uint8_t brightness) {
            if (index < 0 || index >= 64) return;
            const int physical_index = matrix_logical_led_to_physical(index);
            if (physical_index < 0 || physical_index >= 64) return;
            const int offset = physical_index * 3;
            frame_output[offset + 0] = scale8(color.g, brightness);
            frame_output[offset + 1] = scale8(color.r, brightness);
            frame_output[offset + 2] = scale8(color.b, brightness);
        };

        auto set_ui_pixel_physical = [](int index, RGB color, uint8_t brightness) {
            if (index < 0 || index >= 64) return;
            const int offset = index * 3;
            frame_output[offset + 0] = scale8(color.g, brightness);
            frame_output[offset + 1] = scale8(color.r, brightness);
            frame_output[offset + 2] = scale8(color.b, brightness);
        };

        const SettingsPage page = settings_page_state.load(std::memory_order_acquire);
        if (page == SettingsPage::Main) {
            const bool enabled = led_settings_velocity_enabled.load(std::memory_order_acquire);
            const RGB yellow = enabled ? RGB{255, 180, 0} : RGB{24, 17, 0};
            set_ui_pixel(0, yellow, 255);

            const RGB rotation_color = {0, 255, 0};
            const uint8_t inactive_rotation_brightness = 32;
            const RotationMode rotation = settings_rotation();
            const uint8_t rotation_0_brightness = rotation == RotationMode::Deg0 ? 255 : inactive_rotation_brightness;
            const uint8_t rotation_90_brightness = rotation == RotationMode::Deg90 ? 255 : inactive_rotation_brightness;
            const uint8_t rotation_180_brightness = rotation == RotationMode::Deg180 ? 255 : inactive_rotation_brightness;
            const uint8_t rotation_270_brightness = rotation == RotationMode::Deg270 ? 255 : inactive_rotation_brightness;

            set_ui_pixel_physical(19, rotation_color, rotation_0_brightness);
            set_ui_pixel_physical(20, rotation_color, rotation_0_brightness);
            set_ui_pixel_physical(26, rotation_color, rotation_90_brightness);
            set_ui_pixel_physical(34, rotation_color, rotation_90_brightness);
            set_ui_pixel_physical(43, rotation_color, rotation_180_brightness);
            set_ui_pixel_physical(44, rotation_color, rotation_180_brightness);
            set_ui_pixel_physical(29, rotation_color, rotation_270_brightness);
            set_ui_pixel_physical(37, rotation_color, rotation_270_brightness);

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

        const RotationMode rotation = settings_rotation();
        if (!lut_ready || rotation != lut_rotation) {
            for (int i = 0; i < NUM_LEDS; ++i) {
                logical_to_physical_lut[i] = static_cast<int8_t>(
                    matrix_logical_led_to_physical(i));
            }
            lut_rotation = rotation;
            lut_ready = true;
        }
        const uint8_t matrix_effective = scale8(master_brightness, matrix_brightness);
        const uint8_t side_effective = scale8(master_brightness, side_brightness);
        for (size_t i = 0; i < LED_DATA_BYTES; ++i) frame_output[i] = 0;
        for (int i = 0; i < NUM_LEDS; ++i) {
            const int physical_index = logical_to_physical_lut[i];
            if (physical_index < 0 || physical_index >= NUM_LEDS) continue;
            const int offset = physical_index * 3;
            const uint8_t effective = i < 64 ? matrix_effective : side_effective;
            frame_output[offset + 0] = scale8(framebuffer[i].g, effective);
            frame_output[offset + 1] = scale8(framebuffer[i].r, effective);
            frame_output[offset + 2] = scale8(framebuffer[i].b, effective);
        }
    }

    xSemaphoreGive(framebuffer_mutex);
    return true;
}

static void led_task(void *) {
    const rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 1,
        },
    };
    int next_dma_frame = 0;
    int64_t last_start_us = 0;
    int64_t stats_start_us = esp_timer_get_time();
    uint32_t last_completed = 0;
    uint32_t prepare_count = 0;
    uint64_t prepare_total_us = 0;
    uint32_t tx_errors = 0;

    for (;;) {
        // Notifications can come from MIDI updates, timer pacing, and RMT done.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));

        int64_t now = esp_timer_get_time();
        if (now - stats_start_us >= 1000000) {
            const uint32_t done = led_completed_frames.load(std::memory_order_relaxed);
            const uint32_t completed = done - last_completed;
            const int64_t elapsed = now - stats_start_us;
            ESP_LOGI(TAG, "PERF LED done_fps=%lu prepared=%lu avg_prepare_us=%lu tx_err=%lu",
                static_cast<unsigned long>((static_cast<uint64_t>(completed) * 1000000ULL) / elapsed),
                static_cast<unsigned long>(prepare_count),
                static_cast<unsigned long>(prepare_count ? prepare_total_us / prepare_count : 0),
                static_cast<unsigned long>(tx_errors));
            last_completed = done;
            prepare_count = 0;
            prepare_total_us = 0;
            tx_errors = 0;
            stats_start_us = now;
        }

        if (led_rmt_busy.load(std::memory_order_acquire) ||
            !led_dirty.load(std::memory_order_acquire)) {
            continue;
        }

        now = esp_timer_get_time();
        const int64_t until = last_start_us + LED_FRAME_INTERVAL_US - now;
        if (last_start_us != 0 && until > 0) {
            if (led_pacing_timer && !esp_timer_is_active(led_pacing_timer)) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(led_pacing_timer, until));
            }
            continue;
        }

        const int64_t prepare_start = esp_timer_get_time();
        if (!prepare_latest_frame(led_data[next_dma_frame])) {
            continue;
        }
        ++prepare_count;
        prepare_total_us += esp_timer_get_time() - prepare_start;

        led_rmt_busy.store(true, std::memory_order_release);
        const esp_err_t err = rmt_transmit(
            rmt_channel,
            rmt_encoder,
            led_data[next_dma_frame],
            sizeof(led_data[next_dma_frame]),
            &tx_cfg
        );
        if (err == ESP_OK) {
            last_start_us = esp_timer_get_time();
            next_dma_frame ^= 1;
        } else {
            led_rmt_busy.store(false, std::memory_order_release);
            led_dirty.store(true, std::memory_order_release);
            ++tx_errors;
            // Back off on RMT queue error instead of spinning.
            if (led_pacing_timer && !esp_timer_is_active(led_pacing_timer)) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(esp_timer_start_once(led_pacing_timer, 1000));
            }
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
    channel_cfg.mem_block_symbols = 1024;
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

    rmt_tx_event_callbacks_t callbacks = {};
    callbacks.on_trans_done = led_on_rmt_done;
    err = rmt_tx_register_event_callbacks(rmt_channel, &callbacks, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_tx_register_event_callbacks failed: %s", esp_err_to_name(err));
        return;
    }

    err = rmt_enable(rmt_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(err));
        return;
    }

    const BaseType_t task_created = xTaskCreatePinnedToCore(
        led_task,
        "led_refresh",
        4096,
        nullptr,
        5,
        &led_task_handle,
        0
    );
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LED refresh task");
        return;
    }

    esp_timer_create_args_t timer_args = {};
    timer_args.callback = led_timer_callback;
    timer_args.arg = led_task_handle;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.name = "led_pace_3ms";

    err = esp_timer_create(&timer_args, &led_pacing_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed: %s", esp_err_to_name(err));
        return;
    }
    // The pacing timer is one-shot; RMT completion triggers the next refresh.
    led_fill({0, 0, 0});
    led_mark_dirty();
    ESP_LOGI(TAG, "RMT DMA LED driver started at target %u FPS (%lu us period)",
             LED_TARGET_FPS, static_cast<unsigned long>(LED_FRAME_INTERVAL_US));
}

void led_set_pixel(int index, RGB color) {
    if (index < 0 || index >= NUM_LEDS) return;
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    framebuffer[index] = color;
    led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
    notify_led_task();
}

void led_fill(RGB color) {
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    for (auto &pixel : framebuffer) pixel = color;
    led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
    notify_led_task();
}

void led_set_brightness(uint8_t brightness) {
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    g_brightness = brightness;
    led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
    notify_led_task();
}

void led_mark_dirty() {
    led_dirty.store(true, std::memory_order_release);
    notify_led_task();
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
    if (notify) notify_led_task();
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
    notify_led_task();
}

void led_boot_begin() {
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    for (auto &pixel : boot_framebuffer) pixel = RGB{0, 0, 0};
    led_boot_active.store(true, std::memory_order_release);
    led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
    notify_led_task();
}

void led_boot_end() {
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
    led_boot_active.store(false, std::memory_order_release);
    led_dirty.store(true, std::memory_order_release);
    xSemaphoreGive(framebuffer_mutex);
    notify_led_task();
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
    notify_led_task();
}
