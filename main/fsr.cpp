#include "fsr.h"

#include "config.h"
#include "matrix.h"
#include "midi.h"
#include "settings.h"

#include <algorithm>
#include <atomic>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "esp_private/esp_sleep_internal.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "ulp_fsr_keypad.h"
#include "ulp_riscv.h"

static const char *TAG = "fsr";

static constexpr uint16_t LOW_THRESHOLD = 1536;
static constexpr uint16_t HIGH_THRESHOLD = 32767;
static constexpr uint16_t ACTIVATION_OFFSET = 256;
static constexpr uint32_t PAD_DEBOUNCE_MS = 10;
static constexpr uint32_t FN_DEBOUNCE_MS = 20;
static constexpr uint32_t FSR_SCAN_HZ = 240;
static constexpr uint8_t HISTORY_SIZE = 8;
static constexpr uint8_t VELOCITY_SAMPLES = 8;
static constexpr uint8_t MIN_VELOCITY = 1;
static constexpr uint8_t MAX_VELOCITY = 127;
static constexpr uint32_t VELOCITY_MIN_SLOPE = 100;
static constexpr uint32_t VELOCITY_MAX_SLOPE = 10240;

static std::atomic<uint8_t> runtime_min_velocity{MIN_VELOCITY};
static std::atomic<uint8_t> runtime_max_velocity{MAX_VELOCITY};
static std::atomic<uint32_t> runtime_min_slope{VELOCITY_MIN_SLOPE};
static std::atomic<uint32_t> runtime_max_slope{VELOCITY_MAX_SLOPE};

extern const uint8_t ulp_fsr_keypad_bin_start[] asm("_binary_ulp_fsr_keypad_bin_start");
extern const uint8_t ulp_fsr_keypad_bin_end[] asm("_binary_ulp_fsr_keypad_bin_end");

enum class PadState : uint8_t {
    Idle,
    DebouncingPress,
    Active,
    DebouncingRelease,
};

struct PadRuntime {
    PadState state = PadState::Idle;
    uint32_t state_start_ms = 0;
    uint16_t last_history_index = 0;
    uint32_t last_ulp_count = 0;
    uint8_t sample_count = 0;
    uint16_t samples[VELOCITY_SAMPLES]{};
    bool midi_active = false;
    bool suppress_until_release = false;
};

static PadRuntime pads[ROW_NUM][COL_NUM]{};
static adc_oneshot_unit_handle_t adc_handle = nullptr;

static bool fn_raw = false;
static bool fn_stable = false;
static int64_t fn_last_change_us = 0;

static inline uint32_t millis32() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

static inline uint16_t stable_reading(uint8_t x, uint8_t y) {
    auto *samples = reinterpret_cast<volatile uint16_t (*)[COL_NUM]>(&ulp_stable_sample);
    return samples[x][y];
}

static inline uint16_t history_index() {
    return static_cast<uint16_t>(ulp_history_index);
}

static inline uint32_t ulp_scan_count() {
    return static_cast<uint32_t>(ulp_count);
}

static inline uint16_t history_reading(uint8_t index, uint8_t x, uint8_t y) {
    auto *history = reinterpret_cast<volatile uint16_t (*)[COL_NUM][ROW_NUM][COL_NUM]>(&ulp_raw_history);
    return (*history)[index][x][y];
}

static inline uint16_t latest_history_index() {
    return static_cast<uint16_t>((history_index() + HISTORY_SIZE - 1) % HISTORY_SIZE);
}

static void reset_velocity_capture(PadRuntime &runtime) {
    runtime.last_history_index = 0;
    runtime.last_ulp_count = 0;
    runtime.sample_count = 0;
}

static void append_sample(PadRuntime &runtime, uint16_t reading) {
    if (runtime.sample_count >= VELOCITY_SAMPLES) return;
    runtime.samples[runtime.sample_count++] = reading;
}

static void start_velocity_capture(PadRuntime &runtime, uint8_t x, uint8_t y) {
    reset_velocity_capture(runtime);

    const uint16_t latest = latest_history_index();
    const uint16_t previous = static_cast<uint16_t>((latest + HISTORY_SIZE - 1) % HISTORY_SIZE);

    runtime.last_history_index = latest;
    runtime.last_ulp_count = ulp_scan_count();

    append_sample(runtime, history_reading(previous, x, y));
    if (latest != previous) append_sample(runtime, history_reading(latest, x, y));
}

static void capture_new_samples(PadRuntime &runtime, uint8_t x, uint8_t y) {
    const uint32_t current_count = ulp_scan_count();
    const uint32_t delta = current_count - runtime.last_ulp_count;

    if (delta == 0 || runtime.sample_count >= VELOCITY_SAMPLES) return;

    const uint32_t frames = std::min<uint32_t>(delta, HISTORY_SIZE);
    uint16_t index = delta >= HISTORY_SIZE
        ? static_cast<uint16_t>((history_index() + HISTORY_SIZE - frames) % HISTORY_SIZE)
        : static_cast<uint16_t>((runtime.last_history_index + 1) % HISTORY_SIZE);

    for (uint32_t i = 0; i < frames && runtime.sample_count < VELOCITY_SAMPLES; ++i) {
        append_sample(runtime, history_reading(index, x, y));
        runtime.last_history_index = index;
        index = static_cast<uint16_t>((index + 1) % HISTORY_SIZE);
    }

    runtime.last_ulp_count = current_count;
}

static inline uint32_t sum_1_to_n(uint8_t count) {
    return (static_cast<uint32_t>(count) * (count + 1U)) / 2U;
}

static inline uint32_t sum_squares_1_to_n(uint8_t count) {
    return (static_cast<uint32_t>(count) * (count + 1U) * ((count * 2U) + 1U)) / 6U;
}

static uint32_t regression_slope(uint32_t sum_y, uint32_t sum_xy, uint8_t sample_count) {
    if (sample_count == 0) return 0;

    const uint32_t n = static_cast<uint32_t>(sample_count) + 1U;
    const uint32_t sum_x = sum_1_to_n(sample_count);
    const uint32_t sum_x2 = sum_squares_1_to_n(sample_count);
    const uint32_t denominator = (n * sum_x2) - (sum_x * sum_x);

    if (denominator == 0) return sum_y;

    const int64_t numerator =
        static_cast<int64_t>(n) * static_cast<int64_t>(sum_xy) -
        static_cast<int64_t>(sum_x) * static_cast<int64_t>(sum_y);

    if (numerator <= 0) return 0;
    return static_cast<uint32_t>(numerator / denominator);
}

static uint8_t calculate_velocity(const PadRuntime &runtime) {
    if (runtime.sample_count == 0) return 1;

    const uint16_t baseline = runtime.samples[0];
    uint8_t lane1_count = 0;
    uint8_t lane2_count = 0;
    uint32_t lane1_sum_y = 0;
    uint32_t lane1_sum_xy = 0;
    uint32_t lane2_sum_y = 0;
    uint32_t lane2_sum_xy = 0;

    for (uint8_t i = 0; i < runtime.sample_count; ++i) {
        const uint16_t reading = runtime.samples[i];
        const uint16_t adjusted = reading > baseline ? static_cast<uint16_t>(reading - baseline) : 0;

        if ((i & 1U) == 0) {
            ++lane1_count;
            lane1_sum_y += adjusted;
            lane1_sum_xy += static_cast<uint32_t>(lane1_count) * adjusted;
        } else {
            ++lane2_count;
            lane2_sum_y += adjusted;
            lane2_sum_xy += static_cast<uint32_t>(lane2_count) * adjusted;
        }
    }

    const uint32_t slope1 = regression_slope(lane1_sum_y, lane1_sum_xy, lane1_count);
    const uint32_t slope2 = regression_slope(lane2_sum_y, lane2_sum_xy, lane2_count);
    const uint32_t active_lanes = (lane1_count ? 1U : 0U) + (lane2_count ? 1U : 0U);
    const uint32_t slope = active_lanes ? (slope1 + slope2) / active_lanes : 0;

    const uint8_t min_velocity = runtime_min_velocity.load(std::memory_order_acquire);
    const uint8_t max_velocity = runtime_max_velocity.load(std::memory_order_acquire);
    const uint32_t min_slope = runtime_min_slope.load(std::memory_order_acquire);
    const uint32_t max_slope = runtime_max_slope.load(std::memory_order_acquire);

    uint32_t velocity = min_velocity;

    if (slope >= max_slope) {
        velocity = max_velocity;
    } else if (slope > min_slope && max_slope > min_slope) {
        velocity =
            min_velocity +
            ((slope - min_slope) *
             (max_velocity - min_velocity)) /
            (max_slope - min_slope);
    }

    velocity = std::clamp<uint32_t>(
        velocity,
        min_velocity,
        max_velocity
    );
    return static_cast<uint8_t>(velocity);
}

static void suppress_all_active_pads() {
    for (int y = 0; y < ROW_NUM; ++y) {
        for (int x = 0; x < COL_NUM; ++x) {
            PadRuntime &runtime = pads[y][x];
            if (runtime.midi_active) {
                midi_queue_button(matrix_note_at(y, x), false, 0);
                runtime.midi_active = false;
            }
            runtime.suppress_until_release = true;
        }
    }
}

static void process_fn() {
    const int64_t now_us = esp_timer_get_time();
    const bool pressed = gpio_get_level(static_cast<gpio_num_t>(FN_GPIO)) == 0;

    if (pressed != fn_raw) {
        fn_raw = pressed;
        fn_last_change_us = now_us;
    }

    if (fn_raw != fn_stable && (now_us - fn_last_change_us) >= static_cast<int64_t>(FN_DEBOUNCE_MS) * 1000LL) {
        fn_stable = fn_raw;
        if (fn_stable) {
            suppress_all_active_pads();
            settings_toggle_ui();
        }
    }
}

static void process_pad(uint8_t y, uint8_t x, uint32_t now_ms) {
    PadRuntime &runtime = pads[y][x];
    const uint16_t reading = stable_reading(x, y);
    const uint16_t press_threshold = LOW_THRESHOLD + ACTIVATION_OFFSET;
    const bool above_press = reading > press_threshold;
    const bool above_release = reading > LOW_THRESHOLD;

    if (runtime.suppress_until_release) {
        if (!above_release) {
            runtime.suppress_until_release = false;
            runtime.state = PadState::Idle;
            reset_velocity_capture(runtime);
        }
        return;
    }

    switch (runtime.state) {
        case PadState::Idle:
            if (above_press) {
                runtime.state = PadState::DebouncingPress;
                runtime.state_start_ms = now_ms;
                start_velocity_capture(runtime, x, y);
            }
            break;

        case PadState::DebouncingPress:
            if (!above_press) {
                runtime.state = PadState::Idle;
                reset_velocity_capture(runtime);
            } else {
                capture_new_samples(runtime, x, y);
                if (now_ms - runtime.state_start_ms >= PAD_DEBOUNCE_MS) {
                    runtime.state = PadState::Active;

                    if (settings_ui_active()) {
                        const int physical_index = static_cast<int>(y * 8 + x);
                        const int ui_index = matrix_physical_led_to_logical(physical_index);
                        if (ui_index < 0) break;
                        const uint8_t ui_y = static_cast<uint8_t>(ui_index / 8);
                        const uint8_t ui_x = static_cast<uint8_t>(ui_index % 8);

                        if (settings_page() == SettingsPage::Main) {
                            if (physical_index == 19 || physical_index == 20) {
                                settings_set_rotation(RotationMode::Deg0);
                            } else if (physical_index == 26 || physical_index == 34) {
                                settings_set_rotation(RotationMode::Deg90);
                            } else if (physical_index == 43 || physical_index == 44) {
                                settings_set_rotation(RotationMode::Deg180);
                            } else if (physical_index == 29 || physical_index == 37) {
                                settings_set_rotation(RotationMode::Deg270);
                            } else if (ui_y == 0 && ui_x == 0) {
                                settings_toggle_velocity();
                            } else if ((ui_y == 3 || ui_y == 4) && (ui_x == 3 || ui_x == 4)) {
                                settings_open_brightness();
                            } else if (ui_y == 7 && ui_x == 7) {
                                settings_enter_tinyuf2();
                            }
                        } else if (settings_page() == SettingsPage::Brightness) {
                            if (ui_y == 5 && ui_x == 0) {
                                settings_select_brightness_target(BrightnessTarget::Matrix);
                            } else if (ui_y == 5 && ui_x == 1) {
                                settings_select_brightness_target(BrightnessTarget::Side);
                            } else if (ui_y == 6) {
                                settings_set_brightness_step(static_cast<uint8_t>(ui_x + 1));
                            } else if (ui_y == 7) {
                                settings_set_brightness_step(static_cast<uint8_t>(ui_x + 9));
                            }
                        }
                    } else {
                        const uint8_t velocity = settings_velocity_enabled() ? calculate_velocity(runtime) : 127;
                        midi_queue_button(matrix_note_at(y, x), true, velocity);
                        runtime.midi_active = true;
                    }
                }
            }
            break;

        case PadState::Active:
            if (!above_release) {
                runtime.state = PadState::DebouncingRelease;
                runtime.state_start_ms = now_ms;
            }
            break;

        case PadState::DebouncingRelease:
            if (above_release) {
                runtime.state = PadState::Active;
            } else if (now_ms - runtime.state_start_ms >= PAD_DEBOUNCE_MS) {
                runtime.state = PadState::Idle;
                if (runtime.midi_active) {
                    midi_queue_button(matrix_note_at(y, x), false, 0);
                    runtime.midi_active = false;
                }
                reset_velocity_capture(runtime);
            }
            break;
    }
}


uint8_t fsr_min_velocity() {
    return runtime_min_velocity.load(std::memory_order_acquire);
}

uint8_t fsr_max_velocity() {
    return runtime_max_velocity.load(std::memory_order_acquire);
}

uint32_t fsr_min_slope() {
    return runtime_min_slope.load(std::memory_order_acquire);
}

uint32_t fsr_max_slope() {
    return runtime_max_slope.load(std::memory_order_acquire);
}

void fsr_set_velocity_config(uint8_t min_velocity, uint8_t max_velocity, uint32_t min_slope, uint32_t max_slope) {
    min_velocity = std::clamp<uint8_t>(min_velocity, 1, 127);
    max_velocity = std::clamp<uint8_t>(max_velocity, min_velocity, 127);
    min_slope = std::clamp<uint32_t>(min_slope, 100, 10240);
    max_slope = std::clamp<uint32_t>(max_slope, min_slope, 10240);

    runtime_min_velocity.store(min_velocity, std::memory_order_release);
    runtime_max_velocity.store(max_velocity, std::memory_order_release);
    runtime_min_slope.store(min_slope, std::memory_order_release);
    runtime_max_slope.store(max_slope, std::memory_order_release);

    nvs_handle_t h;
    if (nvs_open("settings", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "vel_min", min_velocity);
        nvs_set_u8(h, "vel_max", max_velocity);
        nvs_set_u32(h, "slope_min", min_slope);
        nvs_set_u32(h, "slope_max", max_slope);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void load_velocity_config() {
    uint8_t min_velocity = MIN_VELOCITY;
    uint8_t max_velocity = MAX_VELOCITY;
    uint32_t min_slope = VELOCITY_MIN_SLOPE;
    uint32_t max_slope = VELOCITY_MAX_SLOPE;

    nvs_handle_t h;
    if (nvs_open("settings", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "vel_min", &min_velocity);
        nvs_get_u8(h, "vel_max", &max_velocity);
        nvs_get_u32(h, "slope_min", &min_slope);
        nvs_get_u32(h, "slope_max", &max_slope);
        nvs_close(h);
    }

    min_velocity = std::clamp<uint8_t>(min_velocity, 1, 127);
    max_velocity = std::clamp<uint8_t>(max_velocity, min_velocity, 127);
    min_slope = std::clamp<uint32_t>(min_slope, 100, 10240);
    max_slope = std::clamp<uint32_t>(max_slope, min_slope, 10240);

    runtime_min_velocity.store(min_velocity, std::memory_order_release);
    runtime_max_velocity.store(max_velocity, std::memory_order_release);
    runtime_min_slope.store(min_slope, std::memory_order_release);
    runtime_max_slope.store(max_slope, std::memory_order_release);
}


static void fsr_task(void *) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = std::max<TickType_t>(1, configTICK_RATE_HZ / FSR_SCAN_HZ);

    for (;;) {
        process_fn();
        settings_process();
        const uint32_t now_ms = millis32();

        for (uint8_t y = 0; y < ROW_NUM; ++y) {
            for (uint8_t x = 0; x < COL_NUM; ++x) {
                process_pad(y, x, now_ms);
            }
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

void fsr_init() {
    load_velocity_config();

    gpio_config_t fn_cfg = {};
    fn_cfg.pin_bit_mask = 1ULL << FN_GPIO;
    fn_cfg.mode = GPIO_MODE_INPUT;
    fn_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    fn_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    fn_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&fn_cfg));

    adc_oneshot_unit_init_cfg_t init_cfg = {};
    init_cfg.unit_id = ADC_UNIT_1;
    init_cfg.ulp_mode = ADC_ULP_MODE_RISCV;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t channel_cfg = {};
    channel_cfg.atten = ADC_ATTEN_DB_12;
    channel_cfg.bitwidth = ADC_BITWIDTH_12;

    static constexpr adc_channel_t channels[ROW_NUM] = {
        ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4,
        ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9
    };

    for (adc_channel_t channel : channels) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel, &channel_cfg));
    }

    gpio_config_t drive_cfg = {};
    drive_cfg.pin_bit_mask = 0;
    for (int pin : COL_PINS) drive_cfg.pin_bit_mask |= (1ULL << pin);
    drive_cfg.mode = GPIO_MODE_OUTPUT;
    drive_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    drive_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    drive_cfg.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&drive_cfg));
    for (int pin : COL_PINS) gpio_set_level(static_cast<gpio_num_t>(pin), 0);

    adc_set_hw_calibration_code(ADC_UNIT_1, ADC_ATTEN_DB_12);
    esp_sleep_enable_adc_tsens_monitor(true);

    ESP_LOGI(TAG, "FSR ADC initialized, velocity=%s", settings_velocity_enabled() ? "ON" : "OFF");
}

void fsr_start() {
    ulp_riscv_halt();
    ESP_ERROR_CHECK(ulp_riscv_load_binary(
        ulp_fsr_keypad_bin_start,
        static_cast<size_t>(ulp_fsr_keypad_bin_end - ulp_fsr_keypad_bin_start)
    ));
    ESP_ERROR_CHECK(ulp_riscv_run());

    xTaskCreatePinnedToCore(
        fsr_task,
        "fsr_240hz",
        6144,
        nullptr,
        4,
        nullptr,
        0
    );

    ESP_LOGI(TAG, "ULP FSR scanner started");
}
