#pragma once

#include "sdkconfig.h"

#include <stdio.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

// ============================================================
// I2C
// ============================================================

#ifdef __cplusplus
extern "C" {
#endif

#define SDA_PIN             21
#define SCL_PIN             22
#define I2C_FREQUENCY  400000

#define ADXL345_ADDR      0x53
#define VL6180X_ADDR      0x29


//Function declaration
esp_err_t write_reg8(i2c_master_dev_handle_t dev, uint16_t reg, uint8_t value);
esp_err_t read_reg8(i2c_master_dev_handle_t dev, uint16_t reg, uint8_t *value);
void adxl345_init(void);
void adxl345_read(float *x, float *y, float *z);
void vl6180x_init(void);
int vl6180x_read_distance(void);
bool car_on_ground(void);
void sensors_init(void);

#ifdef __cplusplus
}
#endif