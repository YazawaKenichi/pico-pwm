#include "ros2.h"

#include "gun.h"

#include "std_msgs/msg/int32.h"
#include "std_msgs/msg/float32.h"
#include "geometry_msgs/msg/vector3.h"

rcl_node_t node_;
rcl_allocator_t allocator_;
rclc_support_t support_;
rclc_executor_t executor_;

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
    rcl_ret_t rc;
    rc = rclc_subscription_init_default(&test_subscriber_, &node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/pico/test");
    rclc_executor_add_subscription(&executor_, &test_subscriber_, &test_msg_, test_callback_, ON_NEW_DATA);
    return rc;
}

rcl_ret_t ros2_generate_publisher()
{
    rcl_ret_t rc;
    return rc;
}

