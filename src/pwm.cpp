void RosPwmDriver::timer_callback_(rcl_timer_t *timer_, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher_, &duty_, NULL);
}

void RosPwmDriver::init_pub()
{
}

void RosPwmDriver::init_sub()
{
}

void RosPwmDriver::init_pwm()
{
    gpio_set_function(this->target_pin_, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(this->target_pin_);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);    // ← この辺の数字も外から変えられるようにしたい
    pwm_config_set_wrap(&cfg, 19999);
    pwm_init(slice, &cfg, true);
}

RosPwmDriver::pwm_publisher_init(&node_, "/pico/pwm/GP2", target_pin)
{
    this->target_pin_ = target_pin;
    RosPwmDriver::init_pwm();
}

