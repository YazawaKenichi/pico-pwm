#ifndef __SERVO_H__
#define __SERVO_H__

#include "preload.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include <rcl/rcl.h>
#include "std_msgs/msg/int32.h"
#include "std_msgs//msg/bool.h"
#include "geometry_msgs/msg/vector3.h"

///// 砲塔サーボ /////
#define FUTABA 0
#if FUTABA
/* Futaba S3003 */
#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180
#define SERVO_PULSE_LENGTH_MIN  500
#define SERVO_PULSE_LENGTH_MAX 2000
#define SERVO_RESOLUTION 65536
#define SERVO_HZ 20
#else
#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180
#define SERVO_PULSE_LENGTH_MIN 1000
#define SERVO_PULSE_LENGTH_MAX 2000
#define SERVO_RESOLUTION 65536
#define SERVO_HZ 20
#endif

#define ROLL_IN_MIN -90
#define ROLL_IN_MAX 90
#define ROLL_OUT_MIN 0
#define ROLL_OUT_MAX 180

#define PITCH_IN_MIN -45
#define PITCH_IN_MAX 45
#define PITCH_OUT_MIN 0
#define PITCH_OUT_MAX 90

#define YAW_IN_MIN -90
#define YAW_IN_MAX 90
#define YAW_OUT_MIN 0
#define YAW_OUT_MAX 180

#define LOADING_IN_MIN 0
#define LOADING_IN_MAX 1
#define LOADING_OUT_MIN 0
#define LOADING_OUT_MAX 130

uint16_t servo_deg2level(float);
uint16_t set_servo_pwm(uint32_t, float);
void pose_callback_(const void *);
void pose_timer_callback_(rcl_timer_t *, int64_t);
void roll_callback_(const void *);
void roll_timer_callback_(rcl_timer_t *, int64_t);
void pitch_callback_(const void *);
void pitch_timer_callback_(rcl_timer_t *, int64_t);
void yaw_callback_(const void *);
void yaw_timer_callback_(rcl_timer_t *, int64_t);
void loading_callback_(const void *);
void loading_timer_callback_(rcl_timer_t *, int64_t);
void servo_init();

extern rcl_publisher_t pose_publisher_;
extern rcl_subscription_t pose_subscriber_;
extern geometry_msgs__msg__Vector3 pose_msg_;

extern uint16_t roll_level_;
extern rcl_publisher_t roll_publisher_;
extern rcl_subscription_t roll_subscriber_;
extern std_msgs__msg__Int32 roll_msg_;

extern uint16_t pitch_level_;
extern rcl_publisher_t pitch_publisher_;
extern rcl_subscription_t pitch_subscriber_;
extern std_msgs__msg__Int32 pitch_msg_;

extern uint16_t yaw_level_;
extern rcl_publisher_t yaw_publisher_;
extern rcl_subscription_t yaw_subscriber_;
extern std_msgs__msg__Int32 yaw_msg_;

extern uint16_t loading_level_;
extern rcl_publisher_t loading_publisher_;
extern rcl_subscription_t loading_subscriber_;
extern std_msgs__msg__Bool loading_msg_;

#endif

