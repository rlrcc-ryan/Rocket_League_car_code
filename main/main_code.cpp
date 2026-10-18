// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "main_code.h"   // moet Arduino.h, Bluepad32.h, servos_and_motors_handler.h en sensor_handler.h includen

// ============================================================
// TIMERS
// ============================================================

static uint32_t lastUpdate = 0;

// ============================================================
// JUMP TOESTANDSMACHINE
// ============================================================

enum JumpState { JUMP_IDLE, JUMP_STEP1, JUMP_STEP2, JUMP_STEP3 };

static JumpState jumpState = JUMP_IDLE;
static uint32_t  jumpStepStart = 0;      // tijdstip waarop de huidige stap begon
static bool      hasBeenOnGround = false;
static bool      lastX = false;          // voor het detecteren van een nieuwe knopdruk

// Wachttijden per stap (ms)
#define JUMP_DELAY_STEP1_MS  100   // servo 0 omhoog -> daarna servo 2 en 3 omhoog
#define JUMP_DELAY_STEP2_MS  100   // servo 2 en 3 omhoog -> daarna 2 en 3 terug
#define JUMP_DELAY_STEP3_MS  100   // servo 2 en 3 terug -> daarna servo 0 terug
#define JUMP_GROUND_GRACE    3000  // ms na laatste grondcontact dat springen mag

// ============================================================
// CONTROLLER STATUS
// ============================================================

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

struct ControlState {
    int  x_axis = 0;             // linker joystick X (-511..512)
    int  y_axis = 0;             // linker joystick Y (-511..512)
    int  right_trigger = 0;      // RT (0..1023)
    int  left_trigger = 0;       // LT (0..1023)
    bool right_bumper = false;   // RB
    bool left_bumper = false;    // LB
    bool a = false;
    bool b = false;
    bool x = false;
    bool y = false;
    bool connected = false;
    uint32_t lastOnGround = 0;   // tijdstip van laatste grondcontact
};

static ControlState control;     // laatst bekende controllerwaarden

// ============================================================
// JUMP FUNCTIES
// ============================================================

static bool jump_active() {
    return jumpState != JUMP_IDLE;
}

static uint32_t jump_delay_ms(JumpState state) {
    switch (state) {
        case JUMP_STEP1: return JUMP_DELAY_STEP1_MS;
        case JUMP_STEP2: return JUMP_DELAY_STEP2_MS;
        case JUMP_STEP3: return JUMP_DELAY_STEP3_MS;
        default:         return 0;
    }
}

static void jump_start(uint32_t now) {
    // Alleen starten als er niet al een sprong bezig is
    if (jump_active()) return;

    // Alleen springen als de auto de afgelopen 3 seconden de grond heeft geraakt
    if (!hasBeenOnGround || (now - control.lastOnGround) >= JUMP_GROUND_GRACE) return;

    servo_set_pulse(0, SERVO_SWIVEL_UP_US);
    jumpState = JUMP_STEP1;
    jumpStepStart = now;
}

static void jump_update(uint32_t now) {
    if (!jump_active()) return;
    if (now - jumpStepStart < jump_delay_ms(jumpState)) return;   // stap nog niet klaar

    jumpStepStart = now;

    switch (jumpState) {
        case JUMP_STEP1:
            servo_set_pulse(2, SERVO_CENTER_US + 100);
            servo_set_pulse(3, SERVO_CENTER_US + 100);
            servo_set_pulse(4, SERVO_CENTER_US + 100);
            servo_set_pulse(5, SERVO_CENTER_US + 100);
            jumpState = JUMP_STEP2;
            break;

        case JUMP_STEP2:
            servo_set_pulse(2, SERVO_CENTER_US);
            servo_set_pulse(3, SERVO_CENTER_US);
            servo_set_pulse(4, SERVO_CENTER_US);
            servo_set_pulse(5, SERVO_CENTER_US);
            jumpState = JUMP_STEP3;
            break;

        case JUMP_STEP3:
            servo_set_pulse(0, SERVO_SWIVEL_CENTER_US);
            jumpState = JUMP_IDLE;
            break;

        default:
            break;
    }
}

// Breekt een lopende sprong af en zet de servo's terug in het midden
static void jump_abort() {
    jumpState = JUMP_IDLE;
    servo_set_pulse(0, SERVO_SWIVEL_CENTER_US);
    servo_set_pulse(2, SERVO_CENTER_US);
    servo_set_pulse(3, SERVO_CENTER_US);
    servo_set_pulse(4, SERVO_CENTER_US);
    servo_set_pulse(5, SERVO_CENTER_US);
}

// ============================================================
// THROTTLE FUNCTIONS
// ============================================================

void throttle_control(bool grounded, int right_trigger, int left_trigger, bool right_bumper)
{
    int pulse = SERVO_CENTER_US;

    if (grounded) {
        if (right_bumper) {
            pulse = SERVO_CENTER_US + boost_max_amount;   // boost
        } else {
            int throttle = right_trigger - left_trigger;   // -1023..1023
            if (abs(throttle) < throttle_deadzone) throttle = 0;          // deadzone
            pulse = SERVO_CENTER_US + throttle * 500 / 1023;
        }
    }
    servo_set_pulse(2, pulse);
    servo_set_pulse(5, pulse);
}

// ============================================================
// AIR CONTROL FUNCTIONS
// ============================================================

void air_control(bool grounded, int left_bumper, bool right_bumper, int joystick_x, int joystick_y, bool x_button, bool y_button)
{   
    static int boost_value = 0;
    if (right_bumper) {
        boost_value = boost_max_amount;   // boost
    } else {
        boost_value = 0;
    }

    if (!grounded) {
        // CAR IS IN THE AIR: control yaw pitch and roll with the joystick.
        if (left_bumper) {
            // ADD BOOST AND ROLL ,YAW AND PITCH CONTROL
            


        } else {
            

            if (x_button) {
                // X button pressed: roll left
                
            } else if (y_button) {
                // Y button pressed: roll right
                
            } else {
                // No buttons pressed: center the swivel
                
            }
        }





    }
}


// ============================================================
// BLUEPAD32 CALLBACKS
// ============================================================

// Wordt aangeroepen zodra een nieuwe controller verbindt.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                           ctl->getModelName(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    // Alle controllerwaarden terug naar 0 / false
    control = ControlState();
    hasBeenOnGround = false;
    lastX = false;

    // Lopende sprong afbreken en servo's veilig zetten
    jump_abort();
    servo_set_pulse(0, SERVO_SWIVEL_CENTER_US);
    servo_set_pulse(1, SERVO_CENTER_US);
    servo_set_pulse(2, SERVO_CENTER_US);
    servo_set_pulse(3, SERVO_CENTER_US);
    servo_set_pulse(4, SERVO_CENTER_US);
    servo_set_pulse(5, SERVO_CENTER_US);

    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ============================================================
// CONTROLLER UITLEZEN
// ============================================================

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),
        ctl->dpad(),
        ctl->buttons(),
        ctl->axisX(),
        ctl->axisY(),
        ctl->axisRX(),
        ctl->axisRY(),
        ctl->brake(),
        ctl->throttle(),
        ctl->miscButtons(),
        ctl->gyroX(),
        ctl->gyroY(),
        ctl->gyroZ(),
        ctl->accelX(),
        ctl->accelY(),
        ctl->accelZ()
    );
}

// Bewaart alleen de waarden; de servo's worden in loop() aangestuurd
void processGamepad(ControllerPtr ctl) {
    control.x_axis        = ctl->axisX();
    control.y_axis        = ctl->axisY();
    control.left_trigger  = ctl->brake();      // LT, 0..1023
    control.right_trigger = ctl->throttle();   // RT, 0..1023
    control.left_bumper   = ctl->l1();         // LB
    control.right_bumper  = ctl->r1();         // RB
    control.x             = ctl->x();
    control.y             = ctl->y();
    control.a             = ctl->a();
    control.b             = ctl->b();
    control.connected     = true;
    // dumpGamepad(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

// ============================================================
// ARDUINO SETUP / LOOP
// ============================================================

// Draait op CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n",
                   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    bool startScanning = true;
    BP32.setup(&onConnectedController, &onDisconnectedController, startScanning);

    // Let op: dit wist bij elke start alle gekoppelde controllers.
    // Haal deze regel weg als je niet elke keer opnieuw wilt pairen.
    BP32.forgetBluetoothKeys();

    BP32.enableVirtualDevice(false);
    BP32.enableBLEService(false);
}

// Draait op CPU 1
void loop() {
    BP32.update();
    processControllers();     // vult alleen de globale controlvariabelen

    uint32_t now = millis();
    if (now - lastUpdate >= 20) {   // 50 Hz
        lastUpdate = now;
        
        // Geeft alleen de laatst opgeslagen waarde van de sensortaak terug
        bool grounded = car_on_ground();
        Console.printf("afstand=%d mm, grounded=%d\n", sensors_last_distance(), grounded);
        if (grounded) {
            control.lastOnGround = now;
            hasBeenOnGround = true;
            
        }

        throttle_control(grounded, control.right_trigger, control.left_trigger, control.right_bumper);

        air_control(grounded, control.left_bumper, control.right_bumper, control.x_axis, control.y_axis, control.x, control.y);

        // Sturen (servo 1): alleen aan met controller verbonden en op de grond
        if (control.connected && grounded) {
            servo_set_pulse(1, axisToPulse(control.x_axis));
        } else {
            servo_set_pulse(1, SERVO_CENTER_US);
        }

        // Springen alleen bij een NIEUWE druk op X, niet zolang de knop ingedrukt blijft
        if (control.x && !lastX) {
            jump_start(now);
        }
        lastX = control.x;

        // Sprong stap voor stap uitvoeren (zonder delay)
        jump_update(now);

        // Servo 0, 2 en 3 alleen door de rest van de code laten sturen als er niet gesprongen wordt
        if (!jump_active()) {
            // ... hier komt later je normale besturing van servo 0, 2 en 3 ...
        }
    }

    delay(1);
}