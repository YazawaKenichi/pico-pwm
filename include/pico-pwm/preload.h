#ifndef __PRELOAD_H__
#define __PRELOAD_H__

///// PIN 設定 /////
#define GUN_L_PIN 0     //! GPIO0
#define GUN_R_PIN 1     //! GPIO1

#define ROLL_PIN 3      //! GPIO2
#define PITCH_PIN 5     //! GPIO3

#define YAW_PIN 6       //! GPIO4
#define LOADING_PIN 8   //! GPIO5

#define STEPPER_PIN 9   //! GPIO6

#define UART_TX_PIN 16  //! GPIO21
#define UART_RX_PIN 17  //! GPIO22

#define RESCALE(X, I_MAX, I_MIN, O_MAX, O_MIN) ((O_MAX - O_MIN) * (float) ((X - I_MIN) / (float) (I_MAX - I_MIN)) + O_MIN)
#define HZ2US(HERTZ) (1000 * 1000 * 1 / ((float) HERTZ))

///// デバッグ設定 /////
#define INTEGRATE 1
#define STEPPER_UART 1

#endif

