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
#define jump_max_amount 20 //Max is 1000 // prev 500
#define jump_back_extra_amount 0 // Prev 150
#define jump_preload_amount 20 //Prev 200
#define wall_mode_amount 250
#define WALL_BACK_EXTRA_US       300   // extra motorvermogen boven wall_mode_amount bij vol gas/rem
#define WALL_SWIVEL_RANGE_US     200   // maximale swivel-uitslag t.o.v. SERVO_SWIVEL_DOWN_US

// Wachttijden per stap (ms)
#define JUMP_DELAY_STEP1_MS  200   // servo 0 omhoog -> daarna servo 2 en 3 omhoog
#define JUMP_DELAY_STEP2_MS  350   // servo 2 en 3 omhoog -> daarna 2 en 3 terug
#define JUMP_DELAY_STEP3_MS  100   // servo 2 en 3 terug -> daarna servo 0 terug prev 300
#define JUMP_GROUND_GRACE    3000  // ms na laatste grondcontact dat springen mag

#define PITCH_KP            6.0f    // us per graad fout
#define PITCH_KI            0.5f    // us per graad per seconde
#define PITCH_KD            1.5f    // us per graad per seconde
#define PITCH_OUT_MAX_US    20     // maximale correctie prev 200
#define PITCH_I_MAX_US      80      // anti-windup begrenzing
#define PITCH_PID_SIGN      -1.0f    // -1.0f als de correctie de verkeerde kant op werkt
