cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico_grf250 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_grf250
    main.c
)

pico_enable_stdio_usb(pico_grf250 1)
pico_enable_stdio_uart(pico_grf250 0)

target_link_libraries(pico_grf250 
    pico_stdlib
    hardware_i2c
)

pico_add_extra_outputs(pico_grf250)
