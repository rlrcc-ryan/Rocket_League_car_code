#pragma once
 
#include <stdint.h>
#include <stdio.h>

#include <Arduino.h>
#include "sdkconfig.h"
 
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
#define SERVO_MIN_US     1000
#define SERVO_CENTER_US  1500
#define SERVO_SWIVEL_CENTER_US 1000
#define SERVO_SWIVEL_UP_US  2000
#define SERVO_MAX_US     2000
 
#ifdef __cplusplus
extern "C" {
#endif
 
void servo_init(void);
void servo_set_pulse(uint8_t servo, uint32_t pulse_us);
uint32_t axisToPulse(int axis);
 
#ifdef __cplusplus
}
#endif