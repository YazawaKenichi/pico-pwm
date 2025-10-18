cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_pwm C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

link_directories(external/microros_pico/libmicroros)

add_executable(pico_pwm
    src/main.cpp
    external/microros_pico/pico_uart_transport.c
)

add_subdirectory(external/microros_pico)

target_sources(pico_pwm PRIVATE
    external/microros_pico/pico_uart_transport.c
)

target_link_libraries(pico_pwm
    pico_stdlib
    microros
    hardware_uart
)

target_include_directories(pico_pwm PUBLIC
    external/microros_pico/libmicroros/include
    external/microros_pico
    include/pico-pwm
)

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

pico_enable_stdio_usb(pico_pwm 1)
pico_enable_stdio_uart(pico_pwm 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(pico_pwm)


