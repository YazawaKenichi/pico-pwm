#ifndef __MAIN_H__
#define __MAIN_H__

#include "pico/stdlib.h"
#include <rcl/rcl.h>

///// デバッグ設定 /////
#define INTEGRATE 1
#define STEPPER_UART 1

///// PIN 設定 /////
#define GUN_L_PIN 0     //! GPIO0
#define GUN_R_PIN 1     //! GPIO1

#define ROLL_PIN 2      //! GPIO2
#define PITCH_PIN 3     //! GPIO3

#define YAW_PIN 4       //! GPIO4
#define LOADING_PIN 5   //! GPIO5

#define STEPPER_PIN 6   //! GPIO6

#define UART_TX_PIN 21  //! GPIO21
#define UART_RX_PIN 22  //! GPIO22

///// micro-ROS /////
#define RMW_UROS_MAX_NODES            1
#define RMW_UROS_MAX_PUBLISHERS       8
#define RMW_UROS_MAX_SUBSCRIPTIONS    8
#define RMW_UROS_MAX_SERVICES         0
#define RMW_UROS_MAX_CLIENTS          0
#define RMW_UROS_MAX_TIMERS           8
#define RMW_UROS_MAX_WAIT_SET_ENTRIES 24

///// 砲身ブラシレス /////
#define APPARENTLY_MIN -100.0f
#define APPARENTLY_MAX  100.0f
#define HIGH_TIME_MIN   1020.0f
#define HIGH_TIME_MAX   2020.0f
#define RS2205_PWM_HZ 50.0f
#define RS2205_PWM_RESOLUTION 65536    //! 9765.625 < RESOLUTION < 2500000

///// 砲塔サーボ /////
/* Futaba S3003 */
#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180
#define SERVO_PULSE_LENGTH_MIN  500
#define SERVO_PULSE_LENGTH_MAX 3000
#define SERVO_RESOLUTION 65536
#define SERVO_HZ 50

///// UART /////
#define UART_ID uart0
#define BAUD_RATE 115200

///// Macro /////
#define RESCALE(X, I_MAX, I_MIN, O_MAX, O_MIN) ((O_MAX - O_MIN) * (float) ((X - I_MIN) / (float) (I_MAX - I_MIN)) + O_MIN)
#define HZ2US(HERTZ) (1000 * 1000 * 1 / ((float) HERTZ))

bool check_agent_alive();
void init_ros();
float rs2205_adaptor(float);
uint16_t gun_duty2level(float);
uint16_t set_gun_pwm(uint16_t, float);
void gun_l_callback_(const void *);
void gun_r_callback_(const void *);
void gun_l_timer_callback_(rcl_timer_t *, int64_t);
void gun_r_timer_callback_(rcl_timer_t *, int64_t);
void init_gun_pwm();
uint16_t servo_deg2level(float);
uint16_t set_servo_pwm(uint32_t, float);
void roll_callback_(const void *);
void roll_timer_callback_(rcl_timer_t *, int64_t);
void pitch_callback_(const void *);
void pitch_timer_callback_(rcl_timer_t *, int64_t);
void yaw_callback_(const void *);
void yaw_timer_callback_(rcl_timer_t *, int64_t);
void loading_callback_(const void *);
void loading_timer_callback_(rcl_timer_t *, int64_t);
void init_servo_pwm();
void uart_write_float(float);
void uart_write_string(char *);
void stepper_callback_(const void *);
void stepper_timer_callback_(rcl_timer_t *, int64_t);
void set_step_rate(float);
void init_uart();
void init_stepper();
void init_led();
bool time_update_callback(struct repeating_timer *);
void init_time();

#endif

