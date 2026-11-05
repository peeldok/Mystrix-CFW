cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Mystrix_CFW)

# Apply MIDI FIFO override after component manager fetches esp_tinyusb.
include("${CMAKE_CURRENT_LIST_DIR}/cmake/ensure_midi_fifo.cmake")
