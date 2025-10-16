cmake_minimum_required(VERSION 3.12)

project(pico_pwm C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

pico_sdk_init()

# Search Header Files
target_include_directories(pico_pwm PUBLIC
    external/microros_pico
    include/pico-pwm
)

# Search Library Object Files
link_directories(
    external/microros_pico/libmicroros
)

# Link Libraries
target_link_libraries(pico_pwm
    pico_stdlib
    microros
)

# Link Source Files
add_executable(pico_pwm
    src/main.c
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

pico_enable_stdio_usb(pico_pwm 1)
pico_enable_stdio_uart(pico_pwm 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(pico_pwm)

