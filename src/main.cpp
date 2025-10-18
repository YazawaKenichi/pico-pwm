#include "main.hpp"

#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
extern "C" {
#include "pico_uart_transports.h"
}

Main* Main::instance_ = nullptr;

void Main::timer_callback_(rcl_timer_t *timer_, int64_t last_call_time)
{
    if (!instance_) return;
    rcl_ret_t ret = rcl_publish(&instance_->publisher_, &instance_->msg_, NULL);
    instance_->msg_.data++;
}

Main::Main()
{
    instance_ = this;
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    allocator_ = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        while(1)
        {
        }
    }

    rclc_support_init(&support_, 0, NULL, &allocator_);

    rclc_node_init_default(&node_, "pico_node", "", &support_);
    rclc_publisher_init_default(
        &publisher_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_number_publisher");

    rclc_timer_init_default(
        &timer_,
        &support_,
        RCL_MS_TO_NS(1000),
        Main::timer_callback_);

    rclc_executor_init(&this->executor_, &support_.context, 1, &allocator_);
    rclc_executor_add_timer(&this->executor_, &timer_);

    gpio_put(LED_PIN, 1);

    this->msg_.data = 0;
}

void Main::spin()
{
    rclc_executor_spin_some(&this->executor_, RCL_MS_TO_NS(100));
}

int main()
{
    Main app;
    while (true)
    {
        app.spin();
    }
    return 0;
}

