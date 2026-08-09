idf_component_register(
    SRCS
        "main.cpp"
        "boot_animation.cpp"
        "led.cpp"
        "matrix.cpp"
        "midi.cpp"
        "cfy.cpp"
        "palette.cpp"
        "settings.cpp"
        "fsr.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_timer
        esp_system
        esp_adc
        nvs_flash
        ulp
)

ulp_embed_binary(ulp_fsr_keypad "${CMAKE_CURRENT_LIST_DIR}/ULP/fsr_keypad.c" "fsr.cpp")
