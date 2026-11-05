#include "cdc_debug.h"

#include <stdarg.h>
#include <stdio.h>

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
vprintf_like_t old_vprintf = nullptr;

int async_debug_vprintf(const char *format, va_list args) {
    // Keep UART log output intact (also useful for recovery/debugging).
    va_list uart_args;
    va_copy(uart_args, args);
    const int result = old_vprintf ? old_vprintf(format, uart_args) : 0;
    va_end(uart_args);

    if (log_queue != nullptr) {
        LogLine line{};
        va_list format_args;
        va_copy(format_args, args);
        const int count = vsnprintf(line.text, sizeof(line.text), format, format_args);
        va_end(format_args);
        if (count > 0) {
            line.len = static_cast<uint16_t>(
                count < static_cast<int>(sizeof(line.text)) ? count : sizeof(line.text) - 1
            );
            // Never block the MIDI parser or RMT task if the PC cannot keep up.
            xQueueSend(log_queue, &line, 0);
        }
    }
    return result;
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

    old_vprintf = esp_log_set_vprintf(async_debug_vprintf);
    ESP_LOGI("cdc_debug", "CDC debug available (open new COM at 115200, DTR enabled)");
}
