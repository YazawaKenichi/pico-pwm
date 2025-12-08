#ifndef __ROS2_H__
#define __ROS2_H__

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h"

#include "gun.h"
#include "position.h"
#include "servo.h"

///// micro-ROS /////
#define RMW_UROS_MAX_NODES            1
#define RMW_UROS_MAX_PUBLISHERS       8
#define RMW_UROS_MAX_SUBSCRIPTIONS    8
#define RMW_UROS_MAX_SERVICES         0
#define RMW_UROS_MAX_CLIENTS          0
#define RMW_UROS_MAX_TIMERS           8
#define RMW_UROS_MAX_WAIT_SET_ENTRIES 24

extern bool check_agent_alive();
extern void ros2_init();
extern int ros2_spin();
extern rcl_ret_t ros2_generate_node();
extern rcl_ret_t ros2_generate_subscriber();
extern rcl_ret_t ros2_generate_publisher();

#endif

