cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico-esc C CXX ASM) # ASM is for the PIO code assembly file
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(pico-esc
    controller_test.cpp
    QuadratureEncoder.cpp
    WheelSpeedController.cpp
)

# Pio program
pico_generate_pio_header(pico-esc
    ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder_substep.pio
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(pico-esc
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_gpio
    hardware_sync
    hardware_pwm
    hardware_watchdog
)

# comment enables in tactical software
pico_enable_stdio_usb(pico-esc 1)
pico_enable_stdio_uart(pico-esc 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico-esc)

