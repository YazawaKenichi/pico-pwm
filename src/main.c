#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"

#define UART_ID uart0
#define BAUD_RATE 115200

#define UART_TX_PIN 16   // 物理ピン21
#define UART_RX_PIN 17   // 物理ピン22（今回は未使用でもOK）

volatile int received_value_ = 0;
static volatile int rx_accum_ = 0;
bool led_state_ = 0;

void init_led()
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

void set_led(bool b)
{
    gpio_put(PICO_DEFAULT_LED_PIN, b);
    led_state_ = b;
}

bool get_led()
{
    return led_state_;
}

void on_uart_rx(void)
{
    set_led(!get_led());
    while(uart_is_readable(UART_ID))
    {
        char c = uart_getc(UART_ID);
        if (c >= '0' && c <= '9') {
            rx_accum_ = rx_accum_ * 10 + (c - '0');
        } else if (c == '\n') {
            received_value_ = rx_accum_;
            rx_accum_ = 0;
        } else if (c == '\r') {
            // 無視（CRLF対策）
        } else {
            // 数字以外が来たらリセットしたいならここで rx_accum_=0; でもOK
        }
    }
}

int main()
{
    unsigned long long int count = 0;
    stdio_init_all();
    init_led();

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(UART_ID, true);

    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    while (true)
    {
        int rv = received_value_;

        char buf[128];
        int n = snprintf(buf, sizeof(buf), "%llu Hello from Pico! %d\r\n", count, rv);
        if(n > 0)
        {
            uart_write_blocking(UART_ID, (const uint8_t *) buf, (size_t) n);
        }
        count++;

        sleep_ms(1000);
    }
}

