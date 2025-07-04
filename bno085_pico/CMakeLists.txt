cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico_bno08x C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_bno08x
    main.cpp
)

add_subdirectory(
	BNO08x_Pico_Library
)

target_link_libraries(pico_bno08x
    BNO08x_Pico_Library
)

pico_enable_stdio_usb(pico_bno08x 1)
pico_enable_stdio_uart(pico_bno08x 0)

pico_add_extra_outputs(pico_bno08x)
