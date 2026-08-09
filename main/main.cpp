#include "boot_animation.h"
#include "fsr.h"
#include "led.h"
#include "matrix.h"
#include "midi.h"
#include "palette.h"
#include "settings.h"

#include "esp_log.h"

extern "C" void app_main(void) {
    palette_init();
    led_init();
    settings_init();
    matrix_init();
    midi_init();
    fsr_init();
    fsr_start();
    boot_animation_start();

    ESP_LOGI("Mystrix", "Mystrix CFW v1.0.1: 200 FPS + dual MIDI + ULP FSR + rotation");
}
