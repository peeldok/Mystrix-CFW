#include "perf_diag.h"

#include <atomic>
#include <stdint.h>
#include "esp_log.h"

namespace {
// A sliding (not clock-aligned) 1ms/10ms window for every USB-MIDI event.
// 8192 event timestamps = 32KiB; enough to cover 10ms at USB Full Speed.
// Single producer: TinyUSB task executes tud_midi_rx_cb serially.
constexpr uint32_t RING_SIZE = 8192;
constexpr uint32_t RING_MASK = RING_SIZE - 1;
uint32_t arrival_ring[RING_SIZE]{};
uint32_t arrival_write_index = 0;
uint32_t arrival_tail_1ms = 0;
uint32_t arrival_tail_10ms = 0;
std::atomic<uint32_t> burst_1ms_peak{0};
std::atomic<uint32_t> burst_10ms_peak{0};
std::atomic<uint32_t> ring_overrun_count{0};

constexpr uint32_t QWAIT_BIN_US = 250;  // 0.25ms precision
constexpr uint32_t AGE_BIN_US = 500;    // 0.5ms precision
constexpr int QWAIT_HIST_COUNT = 48;    // beyond 12ms in final bin
constexpr int AGE_HIST_COUNT = 96;      // beyond 48ms in final bin
std::atomic<uint32_t> qwait_hist[QWAIT_HIST_COUNT]{};
std::atomic<uint32_t> start_age_hist[AGE_HIST_COUNT]{};
std::atomic<uint32_t> done_age_hist[AGE_HIST_COUNT]{};

std::atomic<uint32_t> qwait_peak_us{0};
std::atomic<uint32_t> start_age_peak_us{0};
std::atomic<uint32_t> done_age_peak_us{0};
std::atomic<uint32_t> frame_gap_peak_us{0};
std::atomic<uint32_t> note_rewrites{0};
std::atomic<uint32_t> note_batches{0};
std::atomic<uint32_t> frame_from_midi_count{0};

inline void update_peak(std::atomic<uint32_t> &peak, uint32_t observed) {
    uint32_t old = peak.load(std::memory_order_relaxed);
    while (observed > old && !peak.compare_exchange_weak(
        old, observed, std::memory_order_relaxed)) {}
}

inline void histogram_record(std::atomic<uint32_t> *hist,
                             uint32_t count, uint32_t step_us,
                             uint32_t observed) {
    uint32_t index = observed / step_us;
    if (index >= count) index = count - 1;
    hist[index].fetch_add(1, std::memory_order_relaxed);
}

uint32_t histogram_take_p99(std::atomic<uint32_t> *hist,
                            uint32_t count, uint32_t step_us,
                            uint32_t *total_out) {
    uint32_t bins[AGE_HIST_COUNT]{};
    uint32_t total = 0;
    for (uint32_t i = 0; i < count; ++i) {
        bins[i] = hist[i].exchange(0, std::memory_order_relaxed);
        total += bins[i];
    }
    *total_out = total;
    if (!total) return 0;
    const uint32_t wanted = (total * 99 + 99) / 100;
    uint32_t cumul = 0;
    for (uint32_t i = 0; i < count; ++i) {
        cumul += bins[i];
        if (cumul >= wanted) return (i + 1) * step_us;
    }
    return count * step_us;
}
} // namespace

void perf_diag_usb_arrival(uint32_t arrival_us) {
    const uint32_t next = arrival_write_index + 1;
    arrival_ring[arrival_write_index & RING_MASK] = arrival_us;
    arrival_write_index = next;

    // Preserve ring correctness if the impossible-for-USB-FS threshold occurs.
    if (next - arrival_tail_1ms > RING_SIZE) {
        arrival_tail_1ms = next - RING_SIZE;
        ring_overrun_count.fetch_add(1, std::memory_order_relaxed);
    }
    if (next - arrival_tail_10ms > RING_SIZE) {
        arrival_tail_10ms = next - RING_SIZE;
        ring_overrun_count.fetch_add(1, std::memory_order_relaxed);
    }
    while (arrival_tail_1ms != next &&
           static_cast<uint32_t>(arrival_us -
               arrival_ring[arrival_tail_1ms & RING_MASK]) >= 1000) {
        ++arrival_tail_1ms;
    }
    while (arrival_tail_10ms != next &&
           static_cast<uint32_t>(arrival_us -
               arrival_ring[arrival_tail_10ms & RING_MASK]) >= 10000) {
        ++arrival_tail_10ms;
    }
    update_peak(burst_1ms_peak, next - arrival_tail_1ms);
    update_peak(burst_10ms_peak, next - arrival_tail_10ms);
}

void perf_diag_midi_queue_wait(uint32_t wait_us) {
    update_peak(qwait_peak_us, wait_us);
    histogram_record(qwait_hist, QWAIT_HIST_COUNT, QWAIT_BIN_US, wait_us);
}

void perf_diag_note_rewrite() {
    note_rewrites.fetch_add(1, std::memory_order_relaxed);
}

void perf_diag_note_batch_flushed() {
    note_batches.fetch_add(1, std::memory_order_relaxed);
}

void perf_diag_led_frame_submitted(uint32_t arrival_us, uint32_t submit_us) {
    const uint32_t age_us = submit_us - arrival_us;
    update_peak(start_age_peak_us, age_us);
    histogram_record(start_age_hist, AGE_HIST_COUNT, AGE_BIN_US, age_us);
}

void perf_diag_led_frame_done(uint32_t arrival_us, uint32_t done_us) {
    const uint32_t age_us = done_us - arrival_us;
    update_peak(done_age_peak_us, age_us);
    histogram_record(done_age_hist, AGE_HIST_COUNT, AGE_BIN_US, age_us);
    frame_from_midi_count.fetch_add(1, std::memory_order_relaxed);
}

void perf_diag_rmt_interframe_gap(uint32_t gap_us) {
    update_peak(frame_gap_peak_us, gap_us);
}

void perf_diag_log_snapshot() {
    uint32_t qn=0, sn=0, dn=0;
    const uint32_t q99 = histogram_take_p99(qwait_hist, QWAIT_HIST_COUNT, QWAIT_BIN_US, &qn);
    const uint32_t s99 = histogram_take_p99(start_age_hist, AGE_HIST_COUNT, AGE_BIN_US, &sn);
    const uint32_t d99 = histogram_take_p99(done_age_hist, AGE_HIST_COUNT, AGE_BIN_US, &dn);
    ESP_LOGI("peak",
        "USB 1ms_max=%lu 10ms_max=%lu ring_overrun=%lu queue_wait_max_us=%lu queue_wait_p99_le_us=%lu queued=%lu",
        static_cast<unsigned long>(burst_1ms_peak.exchange(0, std::memory_order_relaxed)),
        static_cast<unsigned long>(burst_10ms_peak.exchange(0, std::memory_order_relaxed)),
        static_cast<unsigned long>(ring_overrun_count.exchange(0, std::memory_order_relaxed)),
        static_cast<unsigned long>(qwait_peak_us.exchange(0, std::memory_order_relaxed)),
        static_cast<unsigned long>(q99),
        static_cast<unsigned long>(qn)
    );
    ESP_LOGI("peak",
        "LED tx_start_max_us=%lu tx_start_p99_le_us=%lu tx_done_max_us=%lu tx_done_p99_le_us=%lu gap_max_us=%lu frames=%lu batches=%lu note_rewrites=%lu",
        static_cast<unsigned long>(start_age_peak_us.exchange(0, std::memory_order_relaxed)),
        static_cast<unsigned long>(s99),
        static_cast<unsigned long>(done_age_peak_us.exchange(0, std::memory_order_relaxed)),
        static_cast<unsigned long>(d99),
        static_cast<unsigned long>(frame_gap_peak_us.exchange(0, std::memory_order_relaxed)),
        static_cast<unsigned long>(frame_from_midi_count.exchange(0, std::memory_order_relaxed)),
        static_cast<unsigned long>(note_batches.exchange(0, std::memory_order_relaxed)),
        static_cast<unsigned long>(note_rewrites.exchange(0, std::memory_order_relaxed))
    );
}
