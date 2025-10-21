#include "pwm.hpp"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
extern "C"
{
#include "pico_uart_transports.h"
}

#include <std_msgs/msg/float32.h>

void MainNode::spin()
{
    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
}

bool MainNode::check_agent_alive()
{
    const int TIMEOUT_MS = 1000;
    const uint8_t ATTEMPTS = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(TIMEOUT_MS, ATTEMPTS);
    return ret;
}

void MainNode::publisher_settings()
{
    // ↓ こんな感じでかんたんに publisher を建てられたらかんたんでいいな
    RosPwmDriver pwm_gp2;
    pwm_gp2.pwm_publisher_init(&node_, &executor_, "/pico/pwm/GP2");

    RosPwmDriver pwm_gp3;
    pwm_publisher_init(&node_, &executor_, "/pico/pwm/GP3");

    RosPwmDriver pwm_gp4;
    pwm_gp2.pwm_publisher_init(&node_, &executor_, "/pico/pwm/GP4");

}

MainNode::MainNode()
{
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    while(RCL_RET_OK != MainNode::check_agent_alive())
    {
        //! Agent の生存確認
    }

    rclc_support_init(&support_, 0, NULL, &allocator_);
    rclc_node_init_default(&node_, "/pico_node", "", &support);
    rclc_executor_init(&this->executor_, &support_.context, 1, &allocator_);
    MainNode::publisher_settings();
}

