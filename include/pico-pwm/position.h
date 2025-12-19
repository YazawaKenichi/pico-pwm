#ifndef __POSITION_H__
#define __POSITION_H__

#include "preload.h"

#include <math.h>

#include "rcl/rcl.h"
#include "std_msgs/msg/float32.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "led.h"

///// UART /////
#define UART_ID uart0
#define BAUD_RATE 115200

void uart_write_float(float value);
void uart_write_string(char * message_);
void stepper_callback_(const void * msgin);
void stepper_timer_callback_(rcl_timer_t * timer, int64_t last_call_time);
void set_step_rate(float freq_hz);
void init_uart();
void position_init();
bool get_goal();
void goal_timer_callback_(rcl_timer_t *, int64_t);

extern float stepper_bef_;
extern rcl_publisher_t stepper_publisher_;     // 未使用 値をパブリッシュするときに使える用
extern rcl_subscription_t stepper_subscriber_;
extern std_msgs__msg__Float32 stepper_msg_;

extern bool goaled_;
extern rcl_publisher_t goal_publisher_;

#endif

