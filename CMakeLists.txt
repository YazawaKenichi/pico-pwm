cmake_minimum_required(VERSION 3.12)

# Pico SDK の取り込み（環境変数 PICO_SDK_PATH を利用）
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_pwm_subscriber C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# SDK 初期化
pico_sdk_init()

# micro-ROS ライブラリ配置（libmicroros ディレクトリ想定）
link_directories(external/microros_pico/libmicroros)

# 実行ファイル
add_executable(pico_pwm_subscriber
    src/main.c
    external/microros_pico/pico_uart_transport.c
)

# 依存ライブラリ
# - pico_stdlib / hardware_pwm は PWM 例の構成に倣う
# - microros は micro-ROS 例の構成に倣う
target_link_libraries(pico_pwm_subscriber
    pico_stdlib
    hardware_pwm
    microros
)

# micro-ROS のヘッダ (libmicroros/include) を公開インクルードに追加
target_include_directories(pico_pwm_subscriber PUBLIC
    external/microros_pico
    external/microros_pico/libmicroros/include
)

# 余計な I/O を完全に無効化（USB/UART ともに 0）
pico_enable_stdio_usb(pico_pwm_subscriber 0)
pico_enable_stdio_uart(pico_pwm_subscriber 0)
add_compile_definitions(
    PICO_UART_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
)

# サイズ最適化（任意・小さくしたい場合）
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# UF2 等の生成
pico_add_extra_outputs(pico_pwm_subscriber)


