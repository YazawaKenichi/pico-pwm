void put_led(bool b)
{
    gpio_put(PICO_DEFAULT_LED_PIN, b);
    led_state_ = b;
}

bool get_led()
{
    return led_state_;
}

void init_led()
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

