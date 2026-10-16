#include <stdio.h>
#include <stdint.h>

#include <Arduino.h>
#include "sdkconfig.h"
 
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "servos_and_motors_handler.h"
 
#define SERVO_COUNT      6
 
#define SERVO_MIN_US     1000
#define SERVO_CENTER_US  1500
#define SERVO_MAX_US     2000
 
#define SERVO_FREQ_HZ    50
#define SERVO_PERIOD_US  (1000000UL / SERVO_FREQ_HZ)   // 20.000 us
#define SERVO_RES_BITS   LEDC_TIMER_16_BIT
#define SERVO_DUTY_MAX   (1UL << 16)                    // 65536 stappen

#define AXIS_DEADZONE 40
 
// Pas deze GPIO's aan naar jouw bedrading (servo 0 t/m 5)
static const int servo_gpio[SERVO_COUNT] = {
    14, 15, 16, 17, 18, 19
};
 
// Elke servo krijgt een eigen LEDC-kanaal, alle servo's delen 1 timer
static const ledc_channel_t servo_channel[SERVO_COUNT] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5
};
 
/**
 * Zet de puls van een servo.
 *
 * @param servo    servo nummer 0..5
 * @param pulse_us pulsbreedte in microseconden (1000..2000)
 */
void servo_set_pulse(uint8_t servo, uint32_t pulse_us)
{
    if (servo >= SERVO_COUNT) {
        printf("Ongeldig servo nummer: %u\n", servo);
        return;
    }
 
    // Begrens de puls om de servo te beschermen
    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;
 
    uint32_t duty = (pulse_us * SERVO_DUTY_MAX) / SERVO_PERIOD_US;
 
    ESP_ERROR_CHECK(
        ledc_set_duty(LEDC_LOW_SPEED_MODE, servo_channel[servo], duty)
    );
    ESP_ERROR_CHECK(
        ledc_update_duty(LEDC_LOW_SPEED_MODE, servo_channel[servo])
    );
}
 
/**
 * Initialiseert de timer en alle 6 servo-kanalen.
 * Alle servo's starten op de middenstand.
 */
void servo_init(void)
{
    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode      = LEDC_LOW_SPEED_MODE;
    timer_config.duty_resolution = SERVO_RES_BITS;
    timer_config.timer_num       = LEDC_TIMER_0;
    timer_config.freq_hz         = SERVO_FREQ_HZ;
    timer_config.clk_cfg         = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
 
    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_channel_config_t channel_config = {};
        channel_config.gpio_num   = servo_gpio[i];
        channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
        channel_config.channel    = servo_channel[i];
        channel_config.intr_type  = LEDC_INTR_DISABLE;
        channel_config.timer_sel  = LEDC_TIMER_0;
        channel_config.duty       = 0;
        channel_config.hpoint     = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
 
        servo_set_pulse(i, SERVO_CENTER_US);
    }
}
 
uint32_t axisToPulse(int axis)
{
    // Kleine uitslagen negeren (sticks staan zelden precies op 0)
    if (abs(axis) < AXIS_DEADZONE) {
        axis = 0;
    }

    axis = constrain(axis, -511, 512);
    return map(axis, -511, 512, SERVO_MIN_US, SERVO_MAX_US);
}