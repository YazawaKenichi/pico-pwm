#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#define UART_ID uart0
#define BAUD_RATE 115200

#define UART_TX_PIN 16   // 物理ピン21
#define UART_RX_PIN 17   // 物理ピン22（今回は未使用でもOK）

int main()
{
    stdio_init_all();

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    // RX は今回使わないなら設定しなくてもよい
    // gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    while (true) {
        const char* msg = "Hello from Pico!\r\n";
        uart_write_blocking(UART_ID, (const uint8_t*)msg, strlen(msg));
        sleep_ms(1000);
    }
}

