#pragma once
#include <stdint.h>

// High-resolution performance probes; never print from the MIDI or RMT hot path.
void perf_diag_usb_arrival(uint32_t arrival_us);
void perf_diag_midi_queue_wait(uint32_t wait_us);
void perf_diag_note_rewrite();
void perf_diag_note_batch_flushed();
void perf_diag_led_frame_submitted(uint32_t arrival_us, uint32_t submit_us);
void perf_diag_led_frame_done(uint32_t arrival_us, uint32_t done_us);
void perf_diag_rmt_interframe_gap(uint32_t gap_us);
void perf_diag_log_snapshot();
