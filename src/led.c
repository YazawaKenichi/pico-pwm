#include "led.h"

#include "pico/stdlib.h"

void led_set(bool b)
{
    gpio_put(PICO_DEFAULT_LED_PIN, b);
    led_state_ = b;
}

bool led_get()
{
    return led_state_;
}

void led_init()
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

