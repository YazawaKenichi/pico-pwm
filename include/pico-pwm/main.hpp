#ifndef __MAIN_HPP__
#define __MAIN_HPP__

#include <rcl/rcl.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define LED_PIN 25

class Main
{
    public:
        Main();
        void spin();
        static void timer_callback_(rcl_timer_t*, int64_t);
    protected:
    private:
        static Main* instance_;
        rcl_publisher_t publisher_;
        std_msgs__msg__Int32 msg_;
        rcl_timer_t timer_;
        rcl_node_t node_;
        rcl_allocator_t allocator_;
        rclc_support_t support_;
        rclc_executor_t executor_;
};

#endif

