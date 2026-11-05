#include "cdc_debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <atomic>

#include "esp_timer.h"
#include "led.h"
#include "tinyusb_cdc_acm.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "tusb.h"

namespace {
constexpr uint8_t CDC_PORT = 0;
constexpr size_t LOG_LINE_MAX = 256;
constexpr size_t LOG_QUEUE_LENGTH = 32;
constexpr uint32_t CDC_MAX_WAIT_MS = 30;

struct LogLine {
    uint16_t len;
    char text[LOG_LINE_MAX];
};

QueueHandle_t log_queue = nullptr;
// Normal USB debug messages must never print to synchronous 115200 baud UART
// from the high-priority MIDI parser or LED refresh task.
std::atomic<uint32_t> debug_log_drops{0};
TaskHandle_t benchmark_task_handle = nullptr;

// Triggered only by typing B into the debug COM port.
// This synthetic test bypasses all MIDI to measure actual LED/RMT capacity.
void benchmark_task(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI("bench", "START: 5-second LED-only 3ms stress test, stop MIDI playback first");
        const uint32_t start_frames = led_completed_frame_count();
        const int64_t start_us = esp_timer_get_time();
        TickType_t wake = xTaskGetTickCount();
        uint32_t updates = 0;
        while (esp_timer_get_time() - start_us < 5000000LL) {
            // Color changes on all 96 LEDs force a new 288-byte RMT payload.
            const RGB color = (updates & 1U) ? RGB{0, 0, 22} : RGB{22, 0, 0};
            led_fill(color);
            ++updates;
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(3));
        }
        led_fill({0, 0, 0});
        // Allow the final in-flight RMT transaction to finish before counting.
        vTaskDelay(pdMS_TO_TICKS(10));
        const int64_t elapsed = esp_timer_get_time() - start_us;
        const uint32_t completed = led_completed_frame_count() - start_frames;
        const uint32_t measured_fps =
            static_cast<uint32_t>((static_cast<uint64_t>(completed) * 1000000ULL) / elapsed);
        ESP_LOGI("bench", "FINISHED: updates=%lu completed=%lu measured_fps=%lu target_fps=333",
            static_cast<unsigned long>(updates),
            static_cast<unsigned long>(completed),
            static_cast<unsigned long>(measured_fps));
    }
}


int async_debug_vprintf(const char *format, va_list args) {
    // Log formatting and a single zero-timeout queue enqueue are the only
    // producer work. Never call vprintf()/printf()/UART from a real-time task:
    // a 150-byte message at 115200 baud blocks for ~13ms by itself.
    if (!log_queue) return 0;
    LogLine line{};
    const int count = vsnprintf(line.text, sizeof(line.text), format, args);
    if (count <= 0) return count;
    line.len = static_cast<uint16_t>(
        count < static_cast<int>(sizeof(line.text)) ? count : sizeof(line.text) - 1
    );
    if (xQueueSend(log_queue, &line, 0) != pdTRUE) {
        debug_log_drops.fetch_add(1, std::memory_order_relaxed);
    }
    return count;
}

void cdc_debug_task(void *) {
    LogLine line{};
    for (;;) {
        if (xQueueReceive(log_queue, &line, portMAX_DELAY) != pdTRUE) continue;
        // Discard old output when terminal is closed to keep performance steady.
        if (!tud_cdc_n_connected(CDC_PORT)) continue;
        const TickType_t start = xTaskGetTickCount();
        size_t offset = 0;
        while (offset < line.len && tud_cdc_n_connected(CDC_PORT)) {
            const uint32_t available = tud_cdc_n_write_available(CDC_PORT);
            if (available) {
                const uint32_t remaining = static_cast<uint32_t>(line.len - offset);
                const uint32_t amount = remaining < available ? remaining : available;
                const uint32_t written = tud_cdc_n_write(CDC_PORT, line.text + offset, amount);
                offset += written;
                tud_cdc_n_write_flush(CDC_PORT);
            } else {
                tud_cdc_n_write_flush(CDC_PORT);
                if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(CDC_MAX_WAIT_MS)) break;
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        taskYIELD();
    }
}
} // namespace

void cdc_debug_init() {
    if (log_queue != nullptr) return;
    log_queue = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(LogLine));
    if (!log_queue) {
        ESP_LOGE("cdc_debug", "Unable to allocate CDC debug queue");
        return;
    }
    const BaseType_t created = xTaskCreatePinnedToCore(
        cdc_debug_task, "cdc_debug", 4096, nullptr, 1, nullptr, 1
    );
    if (created != pdPASS) {
        ESP_LOGE("cdc_debug", "Unable to create CDC output task");
        vQueueDelete(log_queue);
        log_queue = nullptr;
        return;
    }

    const BaseType_t bench_ok = xTaskCreatePinnedToCore(
        benchmark_task, "led_bench", 4096, nullptr, 2, &benchmark_task_handle, 0
    );
    if (bench_ok != pdPASS) {
        ESP_LOGE("cdc_debug", "Unable to create LED benchmark task");
    }

    // ROM/boot logs remain available on UART before this point. Runtime ESP-IDF
    // logs are routed only to the low-priority CDC worker to avoid UART stalls.
    esp_log_set_vprintf(async_debug_vprintf);
    ESP_LOGI("cdc_debug", "Async CDC diagnostics enabled: UART runtime mirroring OFF; B=LED-only benchmark");
}

void cdc_debug_rx_callback(int itf, cdcacm_event_t *event) {
    (void)itf;
    (void)event;
    uint8_t input[64];
    size_t got = 0;
    do {
        got = 0;
        if (tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, input, sizeof(input), &got) != ESP_OK) break;
        for (size_t i = 0; i < got; ++i) {
            if ((input[i] == 'B' || input[i] == 'b') && benchmark_task_handle) {
                // CDC callback runs in TinyUSB task context, not an interrupt.
                xTaskNotifyGive(benchmark_task_handle);
            }
        }
    } while (got == sizeof(input));
}

uint32_t cdc_debug_log_drops_take() {
    return debug_log_drops.exchange(0, std::memory_order_relaxed);
}
