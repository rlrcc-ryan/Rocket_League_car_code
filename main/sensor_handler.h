#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// I2C instellingen
// ============================================================

#define SDA_PIN          21
#define SCL_PIN          22
#define I2C_FREQUENCY    400000

#define ADXL345_ADDR     0x53
#define VL6180X_ADDR     0x29

// ============================================================
// Publieke functies (C-linkage, zodat main.c ze ook kan gebruiken)
// ============================================================

#ifdef __cplusplus
extern "C" {
#endif

// Initialiseert de I2C-bus en beide sensoren. Eén keer aanroepen.
void sensors_init(void);

// Start de achtergrondtaak die de afstandssensor continu uitleest.
// Aanroepen NA sensors_init().
void sensors_start_task(void);

// Geeft de laatst bekende status terug (geen I2C-communicatie, direct klaar).
bool car_on_ground(void);

// Laatst gemeten ruwe afstand in mm (-1 bij een fout). Handig voor debuggen.
int sensors_last_distance(void);

// Losse functies (alleen aanroepen als de sensortaak NIET draait).
void adxl345_init(void);
void adxl345_read(float *x, float *y, float *z);
void vl6180x_init(void);
int  vl6180x_read_distance(void);
float sensors_pitch_deg(void);

#ifdef __cplusplus
}
#endif