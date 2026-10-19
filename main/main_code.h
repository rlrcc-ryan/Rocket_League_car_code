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