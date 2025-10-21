#ifndef __PWM_HPP__
#define __PWM_HPP__

class RosPwmDriver()
{
    public:
        RosPwmDriver::pwm_publisher_init(&node_, "/pico/pwm/GP2", target_pin)

        void RosPwmDriver::init_pub()
        void RosPwmDriver::init_sub()
        void RosPwmDriver::init_pwm()

        void RosPwmDriver::timer_callback_(rcl_timer_t *timer_, int64_t last_call_time)

        uint target_pin_;
}

#endif


