#ifndef __MAIN_HPP__
#define __MAIN_HPP__

#include <rcl/rcl.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define LED_PIN 25

class Node
{
    public:
        Node();
        void spin();
        static void timer_callback_(rcl_timer_t*, int64_t);
        static void subscription_callback_(const void* msgin);
    protected:
    private:
        void initNode();
        void setPWM(uint);
        static Node* instance_;
        static uint16_t duty2us(float);

        rcl_publisher_t publisher_;
        std_msgs__msg__Float32 pub_msg_;
        rcl_timer_t timer_;

        rcl_subscription_t subscriber_;
        std_msgs__msg__Float32 sub_msg_;

        rcl_node_t node_;
        rcl_allocator_t allocator_;
        rclc_support_t support_;
        rclc_executor_t executor_;

        float duty_;
};

#endif

