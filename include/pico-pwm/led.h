#ifndef __LED_H__
#define __LED_H__

#include "pico/stdlib.h"
#include "hardware/gpio.h"

void led_set(bool b);
bool led_get();
void led_init();

extern bool led_state_;

#endif

