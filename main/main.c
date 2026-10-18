// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Ricardo Quesada

#include "sdkconfig.h"

#include <stddef.h>
#include <stdio.h>

// BTstack
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>

// Bluepad32
#include <arduino_platform.h>
#include <uni.h>

//Servo
#include "servos_and_motors_handler.h"
//Sensors
#include "sensor_handler.h"


int app_main(void) {

    // Configure console
#ifndef CONFIG_ESP_CONSOLE_UART_NONE
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
#endif	
#endif

// Init servo's
    servo_init();
    vTaskDelay(pdMS_TO_TICKS(500));
    sensors_init();
    sensors_start_task();
    

    // Configure BTstack for ESP32
    btstack_init();

    // Use Bluepad32 Arduino platform
    uni_platform_set_custom(get_arduino_platform());

    // Initialize Bluepad32
    uni_init(0, NULL);

    printf("Bluepad32 gestart!\n");
    printf("Zet je Xbox controller in pairing mode.\n");
    printf("\n");

    // Start Bluetooth event loop
    btstack_run_loop_execute();

#if !CONFIG_AUTOSTART_ARDUINO
    return 0;
#endif
}