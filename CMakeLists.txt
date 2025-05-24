cmake_minimum_required(VERSION 3.13)

# Initialize the SDK
include(pico_sdk_import.cmake)
set(PICOTOOL_FORCE_FETCH_FROM_GIT 1)

project(stopwatch)
# Initialize the SDK
pico_sdk_init()
# Add executable target
add_executable(stopwatch
        main.c
        rtc_time.h
        ssd1306.h
        font5x7.h
)

pico_enable_stdio_usb(stopwatch 1)
pico_enable_stdio_uart(stopwatch 1)

# Link libraries (standard + hardware peripherals)
target_link_libraries(stopwatch
        pico_stdlib
        hardware_spi
        hardware_timer
        hardware_gpio
        hardware_rtc
        hardware_uart
)


# Create UF2 + BIN files for flashing
pico_add_extra_outputs(stopwatch)