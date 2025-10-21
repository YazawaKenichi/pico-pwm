#include "pwm.hpp"

#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define WRAP_12BIT 4095
#define WRAP_16BIT 65535
#define HAMMER_R_PIN 2
#define HAMMER_L_PIN 3

Node* Node::instance_ = nullptr;

void Node::subscription_callback_(const void * msgin)
{
    std_msgs__msg__Float32 * msg = static_cast<const std_msgs__msg__Float32 *>(msgin);    //! -100.0f ~ 100.0f
    pwm_set_gpio_level(target_pin_, pulse);
}

Node::Node()
{
    rclc_publisher_init_default(
        &publisher_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/pico/duty");
    rclc_timer_init_default(
        &timer_,
        &support_,
        RCL_MS_TO_NS(1000),
        Node::timer_callback_);
    rclc_executor_add_timer(&this->executor_, &timer_);

    rclc_subscription_init_default(
            &subscriber_,
            &node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/pico/pwm");
    rclc_executor_add_subscription(
            &this->executor_,
            &this->subscriber_,
            &this->sub_msg_,
            Node::subscription_callback_,
            ON_NEW_DATA);

    duty_ = 0.0f;
    pub_msg_.data = 50.0f;
    sub_msg_.data = 50.0f;
}



