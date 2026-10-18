// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "main_code.h"

//TIMERS
static uint32_t lastUpdate = 0;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

struct ControlState {
    int  x_axis = 0;        // left joystick X
    int  y_axis = 0;        // left joystick Y
    int  right_trigger = 0;     // right trigger
    int  left_trigger = 0;        // left trigger
    int  right_bumper = 0;         // right bumper
    int  left_bumper = 0;          // left bumper
    bool a = false;
    bool b = false;
    bool x = false;
    bool y = false;
    bool connected = false;
    uint32_t lastOnGround = 0;
};

static ControlState control;   // laatst bekende controllerwaarden

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName(), properties.vendor_id,
                           properties.product_id);
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
    
    //Empyt the control state when a controller is disconnected
    control = ControlState(); 
    
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

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
    control.x_axis = ctl->axisX();
    control.y_axis = ctl->axisY();
    control.left_trigger = ctl->brake();
    control.right_trigger = ctl->throttle();
    control.left_bumper = ctl->l1();
    control.right_bumper = ctl->r1 ();
    control.x = ctl->x();
    control.y = ctl->y();  
    control.a = ctl->a();
    control.b = ctl->b();
    control.connected = true;
    //dumpGamepad(ctl);

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

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks, and the default behavior for scanning or not.
    // By default, if the "startScanning" parameter is not passed, it will do the "start scanning".
    // Notice that "Start scanning" will try to auto-connect to devices that are compatible with Bluepad32.
    // E.g: if a Gamepad, keyboard or mouse are detected, it will try to auto connect to them.
    bool startScanning = true;
    BP32.setup(&onConnectedController, &onDisconnectedController, startScanning);

    // Notice that scanning can be stopped / started at any time by calling:
    // BP32.enableNewBluetoothConnections(enabled);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Enables the BLE Service in Bluepad32.
    // This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
    // By default, it is disabled.
    BP32.enableBLEService(false);
}

// Arduino loop function. Runs in CPU 1.
/*void loop() {
    
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    
        
    
    delay(10);
}*/

void loop() {
    //int distance = vl6180x_read_distance();

    //Console.printf("Afstandssensor: %d mm\n", distance);

    // Controller blijven updaten
    BP32.update();
    processControllers();

    uint32_t now = millis();
    if (now - lastUpdate >= 20) {  // 50 Hz
        lastUpdate = now;
        bool grounded = car_on_ground();

        if (control.x) {
            //THE CAR CAN JUMP AT ANY TIME, EVEN WHEN IT IS IN THE AIR> BUT ONLY IF HAS TOUCHED THE GROUND IN THE LAST 3 SECONDS.
            jump();
        }

        if (grounded) {
            //CAR IS ON THE GROUND.
            control.lastOnGround = now;
            servo_set_pulse(1, axisToPulse(control.x_axis)); //ENABLE STEERING.

            

        } else {
            //CAR IS IN THE AIR.
            
            servo_set_pulse(1, SERVO_CENTER_US); //CENTER THE STEERING.


        }
    }

    delay(1);
}

void jump() {
    if (control.lastOnGround < 3000) {
        //CAR HAS BEEN ON THE GROUND FOR LESS THAN 3 SECONDS, SO WE CAN JUMP.
        servo_set_pulse(0, SERVO_CENTER_US + 100);
        delay(100);
        servo_set_pulse(2, SERVO_CENTER_US + 100);
        servo_set_pulse(3, SERVO_CENTER_US + 100);
        delay(100);
        servo_set_pulse(2, SERVO_CENTER_US);
        servo_set_pulse(3, SERVO_CENTER_US);
        delay(100);
        servo_set_pulse(0, SERVO_CENTER_US);
    }
    
}