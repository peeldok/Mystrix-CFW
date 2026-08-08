#include "settings.h"
#include "led.h"

#include <atomic>
#include "esp_timer.h"
#include "esp_private/system_internal.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

static std::atomic<bool> ui_active{false};
static std::atomic<bool> velocity_enabled{true};
static std::atomic<SettingsPage> current_page{SettingsPage::Main};
static std::atomic<BrightnessTarget> brightness_target{BrightnessTarget::Matrix};
static std::atomic<uint8_t> matrix_brightness{255};
static std::atomic<uint8_t> side_brightness{255};
static std::atomic<bool> brightness_save_pending{false};
static std::atomic<int64_t> brightness_save_due_us{0};

static constexpr int64_t BRIGHTNESS_SAVE_DELAY_US = 500000;
static constexpr esp_reset_reason_t APP_REQUEST_UF2_RESET_HINT = static_cast<esp_reset_reason_t>(0x11F2);

static uint8_t step_to_brightness(uint8_t step) {
    if (step < 1) step = 1;
    if (step > 16) step = 16;
    return static_cast<uint8_t>((static_cast<uint16_t>(step) * 255U + 8U) / 16U);
}

static uint8_t brightness_to_step(uint8_t brightness) {
    if (brightness == 0) return 1;
    uint16_t step = (static_cast<uint16_t>(brightness) * 16U + 127U) / 255U;
    if (step < 1) step = 1;
    if (step > 16) step = 16;
    return static_cast<uint8_t>(step);
}

static void refresh_led_ui() {
    led_set_settings_ui(
        ui_active.load(std::memory_order_acquire),
        velocity_enabled.load(std::memory_order_acquire),
        current_page.load(std::memory_order_acquire),
        brightness_target.load(std::memory_order_acquire),
        matrix_brightness.load(std::memory_order_acquire),
        side_brightness.load(std::memory_order_acquire)
    );
}

static void save_velocity(bool enabled) {
    nvs_handle_t h;
    if (nvs_open("settings", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "velocity", enabled ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

static void schedule_brightness_save() {
    brightness_save_due_us.store(esp_timer_get_time() + BRIGHTNESS_SAVE_DELAY_US, std::memory_order_release);
    brightness_save_pending.store(true, std::memory_order_release);
}

static void save_brightness_now() {
    nvs_handle_t h;
    if (nvs_open("settings", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "matrix_br", matrix_brightness.load(std::memory_order_acquire));
    nvs_set_u8(h, "side_br", side_brightness.load(std::memory_order_acquire));
    nvs_commit(h);
    nvs_close(h);
}

bool settings_init() {
    uint8_t saved_velocity = 1;
    uint8_t saved_matrix = 255;
    uint8_t saved_side = 255;

    nvs_handle_t h;
    if (nvs_open("settings", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "velocity", &saved_velocity);
        nvs_get_u8(h, "matrix_br", &saved_matrix);
        nvs_get_u8(h, "side_br", &saved_side);
        nvs_close(h);
    }

    velocity_enabled.store(saved_velocity != 0, std::memory_order_release);
    matrix_brightness.store(saved_matrix, std::memory_order_release);
    side_brightness.store(saved_side, std::memory_order_release);
    current_page.store(SettingsPage::Main, std::memory_order_release);
    brightness_target.store(BrightnessTarget::Matrix, std::memory_order_release);
    refresh_led_ui();
    return true;
}

bool settings_ui_active() {
    return ui_active.load(std::memory_order_acquire);
}

bool settings_velocity_enabled() {
    return velocity_enabled.load(std::memory_order_acquire);
}

SettingsPage settings_page() {
    return current_page.load(std::memory_order_acquire);
}

BrightnessTarget settings_brightness_target() {
    return brightness_target.load(std::memory_order_acquire);
}

uint8_t settings_matrix_brightness() {
    return matrix_brightness.load(std::memory_order_acquire);
}

uint8_t settings_side_brightness() {
    return side_brightness.load(std::memory_order_acquire);
}

uint8_t settings_selected_brightness() {
    return brightness_target.load(std::memory_order_acquire) == BrightnessTarget::Matrix
        ? matrix_brightness.load(std::memory_order_acquire)
        : side_brightness.load(std::memory_order_acquire);
}

uint8_t settings_selected_brightness_step() {
    return brightness_to_step(settings_selected_brightness());
}

void settings_toggle_ui() {
    const bool active = ui_active.load(std::memory_order_acquire);

    if (!active) {
        current_page.store(SettingsPage::Main, std::memory_order_release);
        ui_active.store(true, std::memory_order_release);
    } else if (current_page.load(std::memory_order_acquire) == SettingsPage::Brightness) {
        current_page.store(SettingsPage::Main, std::memory_order_release);
    } else {
        ui_active.store(false, std::memory_order_release);
    }

    refresh_led_ui();
}

void settings_toggle_velocity() {
    const bool next = !velocity_enabled.load(std::memory_order_acquire);
    velocity_enabled.store(next, std::memory_order_release);
    save_velocity(next);
    refresh_led_ui();
}

void settings_open_brightness() {
    if (!ui_active.load(std::memory_order_acquire)) return;
    current_page.store(SettingsPage::Brightness, std::memory_order_release);
    refresh_led_ui();
}

void settings_select_brightness_target(BrightnessTarget target) {
    if (!ui_active.load(std::memory_order_acquire)) return;
    if (current_page.load(std::memory_order_acquire) != SettingsPage::Brightness) return;
    brightness_target.store(target, std::memory_order_release);
    refresh_led_ui();
}

void settings_set_brightness_step(uint8_t step) {
    if (!ui_active.load(std::memory_order_acquire)) return;
    if (current_page.load(std::memory_order_acquire) != SettingsPage::Brightness) return;
    if (step < 1 || step > 16) return;

    const uint8_t value = step_to_brightness(step);
    if (brightness_target.load(std::memory_order_acquire) == BrightnessTarget::Matrix) {
        matrix_brightness.store(value, std::memory_order_release);
    } else {
        side_brightness.store(value, std::memory_order_release);
    }

    schedule_brightness_save();
    refresh_led_ui();
}

void settings_process() {
    if (!brightness_save_pending.load(std::memory_order_acquire)) return;
    if (esp_timer_get_time() < brightness_save_due_us.load(std::memory_order_acquire)) return;

    brightness_save_pending.store(false, std::memory_order_release);
    save_brightness_now();
}

void settings_enter_tinyuf2() {
    if (brightness_save_pending.exchange(false, std::memory_order_acq_rel)) {
        save_brightness_now();
    }

    ui_active.store(false, std::memory_order_release);
    refresh_led_ui();
    led_fill({0, 0, 0});
    led_mark_dirty();

    vTaskDelay(pdMS_TO_TICKS(20));

    (void)esp_reset_reason();
    esp_reset_reason_set_hint(APP_REQUEST_UF2_RESET_HINT);
    esp_restart();
}
