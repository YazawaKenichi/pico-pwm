#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/clocks.h"

//! 砲台左
rcl_publisher_t gun_l_publisher_;
rcl_subscription_t gun_l_subscriber_;
std_msgs__msg__Float32 gun_l_msg_;

//! 砲台右
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
    }
}

void gun_l_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    // pluse が未定義
    // pwm_set_gpio_level(GUN_L_PIN, pulse);
}

void gun_r_callback_(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
    if(msg == NULL)
    {
        return;
    }
    // pluse が未定義
    // pwm_set_gpio_level(GUN_R_PIN, pulse);
}

int main()
{
    ////////// PWM の処理 //////////
    gpio_set_function(GUN_L_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(GUN_L_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, clock_get_hz(clk_sys) / (1000 * 1000));
    pwm_config_set_wrap(&cfg, 9999);
    pwm_init(slice, &cfg, true);

    ////////// micro ROS の処理 //////////
    init_ros();

    rcl_timer_t timer_;
    rcl_node_t node_;
    rcl_allocator_t allocator_ = rcl_get_default_allocator();
    rclc_support_t support_;
    rclc_executor_t executor_;

    ///// ノードの作成 /////
    rclc_support_init(&support_, 0, NULL, &allocator_);
    rclc_node_init_default(&node_, "/pico_node", "", &support_);
    rclc_executor_init(&executor_, &support_.context, 1, &allocator_);

    ///// パブリッシャの作成 /////

    ///// サブスクライバの作成 /////
    //! 砲台左のサブスクライバ
    rclc_subscription_init_default(&gun_l_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/pwm/gun_l");
    rclc_executor_add_subscription(&executor_, &gun_l_subscriber_, &gun_l_msg_, gun_l_callback_, ON_NEW_DATA);
    //! 砲台右のサブスクライバ
    rclc_subscription_init_default(&gun_r_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/pwm/gun_r");
    rclc_executor_add_subscription(&executor_, &gun_r_subscriber_, &gun_r_msg_, gun_r_callback_, ON_NEW_DATA);

    while(true)
    {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
    }

    return 0;
}

