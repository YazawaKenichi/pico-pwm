#include "main.h"

#include <math.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

int main()
{
    init_led();
    // init_time();
    init_stepper();
    ////////// micro ROS の処理 //////////
    init_ros();
    init_gun_pwm();
    init_servo_pwm();

    rcl_node_t node_;
    rcl_allocator_t allocator_ = rcl_get_default_allocator();
    rclc_support_t support_;
    rclc_executor_t executor_;

    ///// ノードの作成 /////
    rclc_support_init(&support_, 0, NULL, &allocator_);
    rclc_node_init_default(&node_, "pico_node", "", &support_);
    rclc_executor_init(&executor_, &support_.context, 16, &allocator_);

    //! ステッピングモータのサブスクライバ
    rcl_ret_t rc;
    rc = rclc_subscription_init_default(&stepper_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/stepper/position/raw");
    rclc_subscription_init_default(&loading_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/loading/deg");
    rclc_executor_add_subscription(&executor_, &loading_subscriber_, &loading_msg_, loading_callback_, ON_NEW_DATA);

    ///// サブスクライバの作成 /////
    //! 砲台左のサブスクライバ
    rclc_subscription_init_default(&gun_l_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/gun/left/pwm/duty");
    rclc_executor_add_subscription(&executor_, &gun_l_subscriber_, &gun_l_msg_, gun_l_callback_, ON_NEW_DATA);
    //! 砲台右のサブスクライバ
    rclc_subscription_init_default(&gun_r_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/gun/right/pwm/duty");
    rclc_executor_add_subscription(&executor_, &gun_r_subscriber_, &gun_r_msg_, gun_r_callback_, ON_NEW_DATA);
    //! 砲塔サーボのサブスクライバ
#if INTEGRATE
    rclc_subscription_init_default(&pose_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/pico/pose");
    rclc_executor_add_subscription(&executor_, &pose_subscriber_, &pose_msg_, pose_callback_, ON_NEW_DATA);
#else
    rclc_subscription_init_default(&roll_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/roll/deg");
    rclc_executor_add_subscription(&executor_, &roll_subscriber_, &roll_msg_, roll_callback_, ON_NEW_DATA);
    rclc_subscription_init_default(&pitch_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/pitch/deg");
    rclc_executor_add_subscription(&executor_, &pitch_subscriber_, &pitch_msg_, pitch_callback_, ON_NEW_DATA);
    rclc_subscription_init_default(&yaw_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/yaw/deg");
    rclc_executor_add_subscription(&executor_, &yaw_subscriber_, &yaw_msg_, yaw_callback_, ON_NEW_DATA);
#endif
    if ( rc != RCL_RET_OK )
    {
        while (true)
        {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(500);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(500);
        }
    }
    rclc_executor_add_subscription(&executor_, &stepper_subscriber_, &stepper_msg_, stepper_callback_, ON_NEW_DATA);

#define PUBLISHER 0
#if PUBLISHER
    ///// パブリッシャの作成 /////
    //! 砲台左のパブリッシャ
    rcl_timer_t gun_l_timer_;
    rclc_publisher_init_default(&gun_l_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/gun/left/pwm/level");
    rclc_timer_init_default(&gun_l_timer_, &support_, RCL_MS_TO_NS(1000), gun_l_timer_callback_);
    rclc_executor_add_timer(&executor_, &gun_l_timer_);
    //! 砲台右のパブリッシャ
    rcl_timer_t gun_r_timer_;
    rclc_publisher_init_default(&gun_r_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/gun/right/pwm/level");
    rclc_timer_init_default(&gun_r_timer_, &support_, RCL_MS_TO_NS(1000), gun_r_timer_callback_);
    rclc_executor_add_timer(&executor_, &gun_r_timer_);

    //! 砲塔サーボのパブリッシャ
    rcl_timer_t pose_timer_;
    rcl_timer_t roll_timer_;
    rcl_timer_t pitch_timer_;
    rcl_timer_t yaw_timer_;
    rcl_timer_t loading_timer_;

#if INTEGRATE
    rclc_publisher_init_default(&pose_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/pose/level");
    rclc_timer_init_default(&pose_timer_, &support_, RCL_MS_TO_NS(1000), pose_timer_callback_);
    rclc_executor_add_timer(&executor_, &pose_timer_);
#else
    rclc_publisher_init_default(&roll_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/roll/level");
    rclc_timer_init_default(&roll_timer_, &support_, RCL_MS_TO_NS(1000), roll_timer_callback_);
    rclc_executor_add_timer(&executor_, &roll_timer_);
    rclc_publisher_init_default(&pitch_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/pitch/level");
    rclc_timer_init_default(&pitch_timer_, &support_, RCL_MS_TO_NS(1000), pitch_timer_callback_);
    rclc_executor_add_timer(&executor_, &pitch_timer_);
    rclc_publisher_init_default(&yaw_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/yaw/level");
    rclc_timer_init_default(&yaw_timer_, &support_, RCL_MS_TO_NS(1000), yaw_timer_callback_);
    rclc_executor_add_timer(&executor_, &yaw_timer_);
#endif //! INTEGRATE

    rclc_publisher_init_default(&loading_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/loading/level");
    rclc_timer_init_default(&loading_timer_, &support_, RCL_MS_TO_NS(1000), loading_timer_callback_);
    rclc_executor_add_timer(&executor_, &loading_timer_);
    //! ステッピングモータのパブリッシャ
    rcl_timer_t stepper_timer_;
    rclc_publisher_init_default(&stepper_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/stepper/position/debug");
    rclc_timer_init_default(&stepper_timer_, &support_, RCL_MS_TO_NS(1000), stepper_timer_callback_);
    rclc_executor_add_timer(&executor_, &stepper_timer_);

#endif //! PUBLISHER

    while(true)
    {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
    }

    return 0;
}

