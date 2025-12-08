#ifndef __GUN_H__
#define __GUN_H__

#include "preload.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include <rcl/rcl.h>
#include "std_msgs/msg/float32.h"
#include "std_msgs/msg/int32.h"

///// 砲身ブラシレス /////
#define APPARENTLY_MIN -100.0f
#define APPARENTLY_MAX  100.0f
#define HIGH_TIME_MIN   1020.0f
#define HIGH_TIME_MAX   2020.0f
#define RS2205_PWM_HZ 50.0f
#define RS2205_PWM_RESOLUTION 65536    //! 9765.625 < RESOLUTION < 2500000

float rs2205_adaptor(float);
uint16_t gun_duty2level(float);
uint16_t set_gun_pwm(uint16_t, float);
void gun_l_callback_(const void *);
void gun_r_callback_(const void *);
void gun_l_timer_callback_(rcl_timer_t *, int64_t);
void gun_r_timer_callback_(rcl_timer_t *, int64_t);
void gun_init();

extern uint16_t gun_l_level_;
extern rcl_publisher_t gun_l_publisher_;
extern rcl_subscription_t gun_l_subscriber_;
extern std_msgs__msg__Float32 gun_l_msg_;

extern uint16_t gun_r_level_;
extern rcl_publisher_t gun_r_publisher_;
extern rcl_subscription_t gun_r_subscriber_;
extern std_msgs__msg__Float32 gun_r_msg_;

#endif

