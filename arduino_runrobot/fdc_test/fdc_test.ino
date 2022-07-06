/*
 * Simple demo, should work with any driver board
 *
 * Connect STEP, DIR as indicated
 *
 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>
#include "BasicStepperDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 400
#define RPM 35
#define STOPPER_PIN 11

#define MICROSTEPS 1

// All the wires needed for full functionality
#define DIR 6
#define STEP 7
//Uncomment line to use enable/disable functionality
#define SLEEP 5

//Uncomment line to use enable/disable functionality
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

void setup() {
    stepper.begin(RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    stepper.setEnableActiveState(LOW);
    pinMode(STOPPER_PIN, INPUT);

}

void loop() {

    while (digitalRead(STOPPER_PIN) == LOW){
      stepper.rotate(-15);
    }
    Serial.println("STOPPER");
    stepper.stop();
    delay(1000);
    stepper.rotate(1000);

    delay(1000);
}
