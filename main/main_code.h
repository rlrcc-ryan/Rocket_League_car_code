#pragma once

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include <servos_and_motors_handler.h>
#include "sensor_handler.h"

#define SERVO_SWIVEL     0
#define SERVO_BACK_1     2
#define SERVO_BACK_2     5
#define SERVO_FRONT_1    3
#define SERVO_FRONT_2    4

#define FRONT_STRENGTH_US    222   // oud: FrontMotorStrength 40 (pitch)
#define FRONT_TORQUE_US      222   // oud: FrontMotorTorqueStrength 40 (yaw)
#define BACK_TORQUE_US       111   // oud: BackMotorTorqueStrength 20 (roll)

#define throttle_deadzone 40
#define boost_max_amount 100
#define jump_max_amount 300 //Max is 1000
#define jump_preload_amount 100
#define wall_mode_amount 100

// Wachttijden per stap (ms)
#define JUMP_DELAY_STEP1_MS  100   // servo 0 omhoog -> daarna servo 2 en 3 omhoog
#define JUMP_DELAY_STEP2_MS  100   // servo 2 en 3 omhoog -> daarna 2 en 3 terug
#define JUMP_DELAY_STEP3_MS  300   // servo 2 en 3 terug -> daarna servo 0 terug
#define JUMP_DELAY_STEP4_MS  100   // servo 0 terug -> daarna sprong voltooid
#define JUMP_GROUND_GRACE    3000  // ms na laatste grondcontact dat springen mag