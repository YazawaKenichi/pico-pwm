#include "servo.h"

#include "std_msgs/msg/float32.h"
#include "std_msgs/msg/int32.h"
#include "geometry_msgs/msg/vector3.h"

#include "hardware/clocks.h"

rcl_publisher_t pose_publisher_;
rcl_subscription_t pose_subscriber_;
geometry_msgs__msg__Vector3 pose_msg_;

uint16_t roll_level_;
rcl_publisher_t roll_publisher_;
rcl_subscription_t roll_subscriber_;
std_msgs__msg__Int32 roll_msg_;

uint16_t pitch_level_;
rcl_publisher_t pitch_publisher_;
rcl_subscription_t pitch_subscriber_;
std_msgs__msg__Int32 pitch_msg_;

uint16_t yaw_level_;
rcl_publisher_t yaw_publisher_;
rcl_subscription_t yaw_subscriber_;
std_msgs__msg__Int32 yaw_msg_;

uint16_t loading_level_;
rcl_publisher_t loading_publisher_;
rcl_subscription_t loading_subscriber_;
std_msgs__msg__Int32 loading_msg_;

uint16_t servo_deg2level(float degree)
{
    float servo_pulse_length = rescale(degree, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_PULSE_LENGTH_MIN, SERVO_PULSE_LENGTH_MAX);
    float servo_duty = 100.0f * servo_pulse_length / (float) HZ2US(SERVO_HZ);
    return SERVO_RESOLUTION * (float) (servo_duty / (float) 100.0f);
}

uint16_t set_servo_pwm(uint32_t pin, float degree)
{
    float degree_;
    float imin, imax;
    float omin, omax;
    switch(pin)
    {
        case ROLL_PIN:
            imin = ROLL_IN_MIN;
            imax = ROLL_IN_MAX;
            omin = ROLL_OUT_MIN;
            omax = ROLL_OUT_MAX;
        case PITCH_PIN:
            imin = PITCH_IN_MIN;
            imax = PITCH_IN_MAX;
            omin = PITCH_OUT_MIN;
            omax = PITCH_OUT_MAX;
        case YAW_PIN:
            imin = YAW_IN_MIN;
            imax = YAW_IN_MAX;
            omin = YAW_OUT_MIN;
            omax = YAW_OUT_MAX;
        case LOADING_PIN:
            imin = LOADING_IN_MAX;
            imax = LOADING_IN_MIN;
            omin = LOADING_OUT_MAX;
            omax = LOADING_OUT_MIN;
        default:
            imin = SERVO_ANGLE_MIN;
            imax = SERVO_ANGLE_MAX;
            omin = SERVO_ANGLE_MIN;
            omax = SERVO_ANGLE_MAX;
    }
    uint16_t level;
#if 0
    degree_ = rescale(degree_, imin, imax, omin, omax);
    degree_ = (degree_ > omax) ? omax : (degree_ < omin) ? omin : degree_;
    level = servo_deg2level(degree_);
#else
    degree_ = (degree > imax) ? imax : (degree_ < imin) ? imin : degree;
    degree_ = rescale(degree_, imin, imax, omin, omax);
    level = servo_deg2level(degree_);
#endif
    pwm_set_gpio_level(pin, level);
    return level;
}

void pose_callback_(const void * msgin)
{
    const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    roll_level_ = set_servo_pwm(ROLL_PIN, msg->x);
    pitch_level_ = set_servo_pwm(PITCH_PIN, msg->y);
    yaw_level_ = set_servo_pwm(YAW_PIN, msg->z);
}

void roll_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    roll_level_ = set_servo_pwm(ROLL_PIN, msg->data);
}

void pose_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    geometry_msgs__msg__Vector3 pub_msg_;
    pub_msg_.x = roll_level_;
    pub_msg_.y = pitch_level_;
    pub_msg_.z = yaw_level_;
    rcl_ret_t rc = rcl_publish(&pose_publisher_, &pub_msg_, NULL);
}

void roll_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = roll_level_;
    rcl_ret_t rc = rcl_publish(&roll_publisher_, &pub_msg_, NULL);
}

void pitch_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    pitch_level_ = set_servo_pwm(PITCH_PIN, msg->data);
}

void pitch_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = pitch_level_;
    rcl_ret_t rc = rcl_publish(&pitch_publisher_, &pub_msg_, NULL);
}

void yaw_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    yaw_level_ = set_servo_pwm(YAW_PIN, msg->data);
}

void yaw_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = yaw_level_;
    rcl_ret_t rc = rcl_publish(&yaw_publisher_, &pub_msg_, NULL);
}

void loading_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    loading_level_ = set_servo_pwm(LOADING_PIN, msg->data);
}

void set_loading(bool tf)
{
    loading_level_ = set_servo_pwm(LOADING_PIN, tf ? LOADING_OUT_MIN : LOADING_OUT_MAX);
    sleep_ms(500);
}

void loading_timer_callback_(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) timer;
    (void) last_call_time;
    std_msgs__msg__Int32 pub_msg_;
    pub_msg_.data = loading_level_;
    rcl_ret_t rc = rcl_publish(&loading_publisher_, &pub_msg_, NULL);
}

void servo_init()
{
    ///// 砲塔 /////
    //! PWM 周波数と分解能の設定
    pwm_config servo_cfg = pwm_get_default_config();
    uint32_t wrap = SERVO_RESOLUTION - 1;
    float clkdiv = (float) clock_get_hz(clk_sys) / (SERVO_HZ * SERVO_RESOLUTION);
    pwm_config_set_wrap(&servo_cfg, wrap);
    pwm_config_set_clkdiv(&servo_cfg, clkdiv);

    //! PWM ピンの設定
    gpio_set_function(ROLL_PIN, GPIO_FUNC_PWM);
    gpio_set_function(PITCH_PIN, GPIO_FUNC_PWM);
    gpio_set_function(YAW_PIN, GPIO_FUNC_PWM);
    gpio_set_function(LOADING_PIN, GPIO_FUNC_PWM);

    uint slice_roll = pwm_gpio_to_slice_num(ROLL_PIN);
    uint slice_pitch = pwm_gpio_to_slice_num(PITCH_PIN);
    uint slice_yaw = pwm_gpio_to_slice_num(YAW_PIN);
    uint slice_loading = pwm_gpio_to_slice_num(LOADING_PIN);

    pwm_init(slice_roll, &servo_cfg, true);
    pwm_init(slice_pitch, &servo_cfg, true);
    pwm_init(slice_yaw, &servo_cfg, true);
    pwm_init(slice_loading, &servo_cfg, true);

    roll_level_ = set_servo_pwm(ROLL_PIN, (ROLL_OUT_MIN + ROLL_OUT_MAX) / 2);
    pitch_level_ = set_servo_pwm(PITCH_PIN, (PITCH_OUT_MIN + PITCH_OUT_MAX) / 2);
    yaw_level_ = set_servo_pwm(YAW_PIN, (YAW_OUT_MIN + YAW_OUT_MAX) / 2);
    loading_level_ = set_servo_pwm(LOADING_PIN, LOADING_OUT_MIN);

    pwm_set_enabled(slice_roll, true);
    pwm_set_enabled(slice_pitch, true);
    pwm_set_enabled(slice_yaw, true);
    pwm_set_enabled(slice_loading, true);
}

