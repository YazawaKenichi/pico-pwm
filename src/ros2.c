#include "ros2.h"

#include "gun.h"
#include "servo.h"
#include "position.h"

#include "std_msgs/msg/int32.h"
#include "std_msgs/msg/float32.h"
#include "geometry_msgs/msg/vector3.h"

rcl_node_t node_;
rcl_allocator_t allocator_;
rclc_support_t support_;
rclc_executor_t executor_;

rcl_timer_t goal_timer_;
rcl_timer_t gun_l_timer_;
rcl_timer_t gun_r_timer_;
rcl_timer_t pose_timer_;
rcl_timer_t roll_timer_;
rcl_timer_t pitch_timer_;
rcl_timer_t yaw_timer_;
rcl_timer_t loading_timer_;
rcl_timer_t stepper_timer_;

// Agent の生存確認
bool check_agent_alive()
{
    const int TIMEOUT_MS = 1000;
    const uint8_t ATTEMPTS = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(TIMEOUT_MS, ATTEMPTS);
    return (ret == RCL_RET_OK);
}

void ros2_init()
{
    allocator_ = rcl_get_default_allocator();
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open, pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);
    while(!check_agent_alive())
    {
        //! Agent から応答が返ってくるまで待機
        sleep_ms(100);
    }

    ros2_generate_node();
    ros2_generate_subscriber();
    ros2_generate_publisher();
}

int ros2_spin()
{
    set_loading(get_trigger());
    rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
    return 0;
}

rcl_ret_t ros2_generate_node()
{
    rcl_ret_t rc;
    ///// ノードの作成 /////
    rclc_support_init(&support_, 0, NULL, &allocator_);
    rclc_node_init_default(&node_, "pico_node", "", &support_);
    rclc_executor_init(&executor_, &support_.context, 16, &allocator_);
    return rc;
}

rcl_ret_t ros2_generate_subscriber()
{
    //! ステッピングモータのサブスクライバ
    rcl_ret_t rc;
    rclc_subscription_init_default(&stepper_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/stepper/position/raw");
    rclc_executor_add_subscription(&executor_, &stepper_subscriber_, &stepper_msg_, stepper_callback_, ON_NEW_DATA);

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
    rclc_subscription_init_default(&loading_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/loading/deg");
    rclc_executor_add_subscription(&executor_, &loading_subscriber_, &loading_msg_, loading_callback_, ON_NEW_DATA);

    return rc;
}

rcl_ret_t ros2_generate_publisher()
{
    rclc_publisher_init_default(&goal_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/pico/stepper/goal");
    rclc_timer_init_default(&goal_timer_, &support_, RCL_MS_TO_NS(1000), goal_timer_callback_);
    rclc_executor_add_timer(&executor_, &goal_timer_);

#define PUBLISHER 0
    rcl_ret_t rc;
#if PUBLISHER
    rclc_publisher_init_default(&gun_l_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/gun/left/pwm/level");
    rclc_timer_init_default(&gun_l_timer_, &support_, RCL_MS_TO_NS(1000), gun_l_timer_callback_);
    rclc_executor_add_timer(&executor_, &gun_l_timer_);
    rclc_publisher_init_default(&gun_r_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/pico/gun/right/pwm/level");
    rclc_timer_init_default(&gun_r_timer_, &support_, RCL_MS_TO_NS(1000), gun_r_timer_callback_);
    rclc_executor_add_timer(&executor_, &gun_r_timer_);
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
    rclc_publisher_init_default(&stepper_publisher_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/stepper/position/debug");
    rclc_timer_init_default(&stepper_timer_, &support_, RCL_MS_TO_NS(1000), stepper_timer_callback_);
    rclc_executor_add_timer(&executor_, &stepper_timer_);
#endif //! PUBLISHER
    return rc;
}

