#ifndef __MAIN_H__
#define __MAIN_H__

#include "pico/stdlib.h"

///// 砲身ブラシレス /////
#define GUN_L_PIN 2
#define GUN_R_PIN 3

#define APPARENTLY_MIN -100.0f
#define APPARENTLY_MAX  100.0f
#define HIGH_TIME_MIN   1020.0f
#define HIGH_TIME_MAX   2020.0f
#define RS2205_PWM_HZ 50.0f
#define RS2205_PWM_RESOLUTION 65536    //! 9765.625 < RESOLUTION < 2500000

#define RESCALE(X, I_MAX, I_MIN, O_MAX, O_MIN) ((O_MAX - O_MIN) * (float) ((X - I_MIN) / (float) (I_MAX - I_MIN)) + O_MIN)
#define HZ2US(HERTZ) (1000 * 1000 * 1 / ((float) HERTZ))

///// 砲塔サーボ /////
#define SERVO_PIN 4

/* Futaba S3003 */
#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180
#define SERVO_PULSE_LENGTH_MIN  500
#define SERVO_PULSE_LENGTH_MAX 3000
#define SERVO_RESOLUTION 65536
#define SERVO_HZ 50

bool check_agent_alive();
void init_ros();
float rs2205_adaptor(float);
uint16_t gun_duty2level(float);
uint16_t set_gun_pwm(uint16_t, float);
void gun_l_callback_(const void *);
void gun_r_callback_(const void *);
void gun_l_timer_callback_();
void gun_r_timer_callback_();
void init_gun_pwm();
uint16_t servo_deg2level(float);
uint16_t set_servo_pwm(uint32_t, float);
void servo_callback_(const void *);
void servo_timer_callback_();
void init_servo_pwm();

#endif

