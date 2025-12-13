#include "gun.h"

#include "hardware/clocks.h"

#include <rcl/rcl.h>

uint16_t gun_l_level_;
rcl_publisher_t gun_l_publisher_;
rcl_subscription_t gun_l_subscriber_;
std_msgs__msg__Float32 gun_l_msg_;

uint16_t gun_r_level_;
rcl_publisher_t gun_r_publisher_;
rcl_subscription_t gun_r_subscriber_;
std_msgs__msg__Float32 gun_r_msg_;


float rs2205_adaptor(float ad)
{
    uint32_t high_time = RESCALE(ad, APPARENTLY_MAX, APPARENTLY_MIN, HIGH_TIME_MAX, HIGH_TIME_MIN);
    //! ↓ すでに -100 ~ 100 でクリッピングされた値を引数にすれば正直ここは素通りするはず
    if(high_time > HIGH_TIME_MAX)
    {
        high_time = HIGH_TIME_MAX;
    }
    if(high_time < HIGH_TIME_MIN)
    {
        high_time = HIGH_TIME_MIN;
    }
    return (100.0f * high_time) / HZ2US(RS2205_PWM_HZ);
}

uint16_t gun_duty2level(float apparently_duty)
{
    //! 上限と下限
    if(apparently_duty < APPARENTLY_MIN)
    {
        apparently_duty = APPARENTLY_MIN;
    }
    if(apparently_duty > APPARENTLY_MAX)
    {
        apparently_duty = APPARENTLY_MAX;
    }
    //! -100 ~ 100 から 実際のデューティ比 5.1% ~ 10.1% に変換
    float realistic_duty = rs2205_adaptor(apparently_duty);
    //! 0 < level < wrap
    return (float) RS2205_PWM_RESOLUTION * (realistic_duty / 100.0f);
}

uint16_t set_gun_pwm(uint16_t pin, float duty)
{
    uint16_t level = gun_duty2level(duty);
    pwm_set_gpio_level(pin, level);
    return level;
}

void gun_l_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    gun_l_level_ = set_gun_pwm(GUN_L_PIN, 1 * msg->data);
}

void gun_r_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    gun_r_level_ = set_gun_pwm(GUN_R_PIN, -1 * msg->data);
}

void gun_l_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = gun_l_level_;
    rcl_ret_t rc = rcl_publish(&gun_l_publisher_, &pub_msg_, NULL);
}

void gun_r_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = gun_r_level_;
    rcl_ret_t rc = rcl_publish(&gun_r_publisher_, &pub_msg_, NULL);
}

void gun_init()
{
    ///// 砲台 /////
    //! PWM 周波数と分解能の設定
    pwm_config gun_cfg = pwm_get_default_config();
    uint32_t wrap = RS2205_PWM_RESOLUTION - 1;
    float clkdiv = (float) clock_get_hz(clk_sys) / (RS2205_PWM_HZ * RS2205_PWM_RESOLUTION);
    pwm_config_set_wrap(&gun_cfg, wrap);
    pwm_config_set_clkdiv(&gun_cfg, clkdiv);
    //! PWM ピンの設定
    gpio_set_function(GUN_L_PIN, GPIO_FUNC_PWM);
    gpio_set_function(GUN_R_PIN, GPIO_FUNC_PWM);
    uint slice_left = pwm_gpio_to_slice_num(GUN_L_PIN);
    uint slice_right = pwm_gpio_to_slice_num(GUN_R_PIN);
    pwm_init(slice_left, &gun_cfg, true);
    pwm_init(slice_right, &gun_cfg, true);
    pwm_set_enabled(slice_left, true);
    pwm_set_enabled(slice_right, true);
    gun_r_level_ = set_gun_pwm(GUN_R_PIN, 0);
    gun_l_level_ = set_gun_pwm(GUN_L_PIN, 0);
    sleep_ms(3000);
}

