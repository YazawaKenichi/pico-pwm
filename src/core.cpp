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
    // ↓ こんな感じでかんたんに publisher を建てられたらいいな
    // pwm_publisher_init(&node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/pwm/GP2");
}

bool MainNode::check_agent_alive()
{
    const int TIMEOUT_MS = 1000;
    const uint8_t ATTEMPTS = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(TIMEOUT_MS, ATTEMPTS);
    return ret;
}

void MainNode::spin()
{
    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
}


