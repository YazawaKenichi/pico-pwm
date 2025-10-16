// main.c
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico_uart_transports.h"

static volatile float latest_value = 0.0f;
static volatile bool  has_value    = false;

static rcl_subscription_t subscriber;
static std_msgs__msg__Float32 msg_in;

// ------------------------------
// Callback: called when a Float32 arrives
// ------------------------------
static void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * m = (const std_msgs__msg__Float32 *)msgin;
    // accept only -100.0f .. +100.0f
    if (m->data >= -100.0f && m->data <= 100.0f) {
        latest_value = m->data;
        has_value = true;
    }
    // else: ignore out-of-range values (keeps previous latest_value)
}

// ------------------------------
// Entry point
// ------------------------------
int main(void)
{
    // Use UART as custom transport (same as micro_ros_raspberrypi_pico_sdk examples)
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Optional: wait for agent (up to ~120 seconds)
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK) {
        // Agent unreachable -> exit early (no printf per request)
        return (int)ret;
    }

    // Allocators / support / node
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "pico_node", "", &support);

    // Subscriber: std_msgs/Float32 on "pico/float32"
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "pico/float32"
    );

    // Executor with 1 handle (the subscription)
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
        &executor, &subscriber, &msg_in, &subscription_callback, ON_NEW_DATA
    );

    // Main loop: spin for new data (no timers/LED/printf)
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // (Unreached in typical embedded loops)
    // rclc_executor_fini(&executor);
    // rcl_subscription_fini(&subscriber, &node);
    // rcl_node_fini(&node);
    return 0;
}


