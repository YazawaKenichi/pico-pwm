#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

#define GUN_L_PIN 2
#define GUN_R_PIN 3

#define APPARENTLY_MIN -100.0f
#define APPARENTLY_MAX  100.0f
#define HIGH_TIME_MIN   1020.0f
#define HIGH_TIME_MAX   2020.0f
#define RS2205_PWM_HZ 50.0f
#define RS2205_PWM_RESOLUTION 65536    //! 9765.625 < RESOLUTION < 2500000

#define HZ2US(HERTZ) (1000 * 1000 * 1 / ((float) HERTZ))

//! 砲台左
uint16_t gun_l_level_;
rcl_publisher_t gun_l_publisher_;
rcl_subscription_t gun_l_subscriber_;
std_msgs__msg__Float32 gun_l_msg_;

//! 砲台右
uint16_t gun_r_level_;
rcl_publisher_t gun_r_publisher_;
rcl_subscription_t gun_r_subscriber_;
std_msgs__msg__Float32 gun_r_msg_;

// Agent の生存確認
bool check_agent_alive()
{
    const int TIMEOUT_MS = 1000;
    const uint8_t ATTEMPTS = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(TIMEOUT_MS, ATTEMPTS);
    return ret;
}

///// 初期化関数 /////
void init_ros()
{
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);
    while(RCL_RET_OK != check_agent_alive())
    {
        //! Agent から応答が返ってくるまで待機
        sleep_ms(100);
    }
}

float rs2205_adaptor(float ad)
{
    uint32_t high_time = (HIGH_TIME_MAX - HIGH_TIME_MIN) * (float) ((ad - APPARENTLY_MIN) / (float) (APPARENTLY_MAX - APPARENTLY_MIN)) + HIGH_TIME_MIN;
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

uint16_t duty2level(float apparently_duty)
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
    uint16_t level = duty2level(duty);
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

void gun_l_timer_callback_()
{
    std_msgs__msg__Float32 pub_msg_;
    pub_msg_.data = gun_l_level_;
    rcl_publish(&gun_l_publisher_, &pub_msg_, NULL);
}

void gun_r_timer_callback_()
{
    std_msgs__msg__Float32 pub_msg_;
    pub_msg_.data = gun_r_level_;
    rcl_publish(&gun_r_publisher_, &pub_msg_, NULL);
}

int main()
{
    ////////// PWM の処理 //////////
    //! PWM 周波数と分解能の設定
    pwm_config cfg = pwm_get_default_config();
    uint32_t wrap = RS2205_PWM_RESOLUTION - 1;
    float clkdiv = (float) clock_get_hz(clk_sys) / (RS2205_PWM_HZ * RS2205_PWM_RESOLUTION);
    pwm_config_set_wrap(&cfg, wrap);
    pwm_config_set_clkdiv(&cfg, clkdiv);
    //! PWM ピンの設定
    gpio_set_function(GUN_L_PIN, GPIO_FUNC_PWM);
    gpio_set_function(GUN_R_PIN, GPIO_FUNC_PWM);
    uint slice_left = pwm_gpio_to_slice_num(GUN_L_PIN);
    uint slice_right = pwm_gpio_to_slice_num(GUN_R_PIN);
    pwm_init(slice_left, &cfg, true);
    pwm_init(slice_right, &cfg, true);
    gun_l_level_ = set_gun_pwm(GUN_L_PIN, 0);
    gun_r_level_ = set_gun_pwm(GUN_R_PIN, 0);

    ////////// micro ROS の処理 //////////
    init_ros();

    rcl_timer_t gun_l_timer_;
    rcl_timer_t gun_r_timer_;
    rcl_node_t node_;
    rcl_allocator_t allocator_ = rcl_get_default_allocator();
    rclc_support_t support_;
    rclc_executor_t executor_;

    ///// ノードの作成 /////
    rclc_support_init(&support_, 0, NULL, &allocator_);
    rclc_node_init_default(&node_, "pico_node", "", &support_);
    rclc_executor_init(&executor_, &support_.context, 4, &allocator_);

    ///// パブリッシャの作成 /////
    //! 砲台左のパブリッシャ
    rclc_publisher_init_default(&gun_l_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/gun/left/pwm/level");
    rclc_timer_init_default(&gun_l_timer_, &support_, RCL_MS_TO_NS(1000), gun_l_timer_callback_);
    rclc_executor_add_timer(&executor_, &gun_l_timer_);
    //! 砲台右のパブリッシャ
    rclc_publisher_init_default(&gun_r_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/gun/right/pwm/level");
    rclc_timer_init_default(&gun_r_timer_, &support_, RCL_MS_TO_NS(1000), gun_r_timer_callback_);
    rclc_executor_add_timer(&executor_, &gun_r_timer_);

    ///// サブスクライバの作成 /////
    //! 砲台左のサブスクライバ
    rclc_subscription_init_default(&gun_l_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/gun/left/pwm/duty");
    rclc_executor_add_subscription(&executor_, &gun_l_subscriber_, &gun_l_msg_, gun_l_callback_, ON_NEW_DATA);
    //! 砲台右のサブスクライバ
    rclc_subscription_init_default(&gun_r_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/gun/right/pwm/duty");
    rclc_executor_add_subscription(&executor_, &gun_r_subscriber_, &gun_r_msg_, gun_r_callback_, ON_NEW_DATA);

    while(true)
    {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
    }

    return 0;
}

