#include "main.hpp"

#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"

extern "C" {
#include "pico_uart_transports.h"
}

#define WRAP_12BIT 4095
#define WRAP_16BIT 65535
#define HAMMER_R_PIN 2
#define HAMMER_L_PIN 3

Node* Node::instance_ = nullptr;

void Node::timer_callback_(rcl_timer_t *timer_, int64_t last_call_time)
{
    if (!instance_) return;
    instance_->pub_msg_.data = instance_->duty_;    //! 0.0f ~ 1.0f
    rcl_ret_t ret = rcl_publish(&instance_->publisher_, &instance_->pub_msg_, NULL);
}

uint16_t Node::duty2us(float duty)
{
}

void Node::subscription_callback_(const void * msgin)
{
    if (!msgin) return;
    const std_msgs__msg__Float32 * msg = static_cast<const std_msgs__msg__Float32 *>(msgin);    //! -100.0f ~ 100.0f
    float duty = 50 + (msg->data / 2);  //! 0.0f ~ 100.0f

    instance_->duty_ = duty / 100.0f;   //! 0.0f ~ 1.0f

    uint16_t pulse = Node::duty2us(duty);

    pwm_set_gpio_level(HAMMER_R_PIN, pulse);
    pwm_set_gpio_level(HAMMER_L_PIN, pulse);
}

void Node::setPWM(uint target_pin)
{
    gpio_set_function(target_pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(target_pin);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);
    pwm_config_set_wrap(&cfg, 19999);
    pwm_init(slice, &cfg, true);
}

void Node::initNode()
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
}

Node::Node()
{
    Node::initNode();

    Node::setPWM(HAMMER_R_PIN);
    Node::setPWM(HAMMER_L_PIN);

    rclc_publisher_init_default(
        &publisher_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/pico/duty");

    rclc_subscription_init_default(
            &subscriber_,
            &node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "/pico/pwm");

    rclc_timer_init_default(
        &timer_,
        &support_,
        RCL_MS_TO_NS(1000),
        Node::timer_callback_);

    rclc_executor_init(&this->executor_, &support_.context, 2, &allocator_);
    rclc_executor_add_timer(&this->executor_, &timer_);
    rclc_executor_add_subscription(
            &this->executor_,
            &this->subscriber_,
            &this->sub_msg_,
            Node::subscription_callback_,
            ON_NEW_DATA);

    gpio_put(LED_PIN, 1);

    duty_ = 0.0f;
    pub_msg_.data = 50.0f;
    sub_msg_.data = 50.0f;
}

void Node::spin()
{
    rclc_executor_spin_some(&this->executor_, RCL_MS_TO_NS(100));
}

int main()
{
    Node node;
    while (true)
    {
        node.spin();
    }
    return 0;
}

