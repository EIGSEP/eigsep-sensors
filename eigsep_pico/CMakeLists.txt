cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico_T_ctrl C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_T_ctrl main.c hbridge_peltier.c read_temp.c)

target_link_libraries(pico_T_ctrl pico_stdlib hardware_pwm hardware_adc pico_multicore)

pico_enable_stdio_usb(pico_T_ctrl 1)
pico_enable_stdio_uart(pico_T_ctrl 0)

pico_add_extra_outputs(pico_T_ctrl)
