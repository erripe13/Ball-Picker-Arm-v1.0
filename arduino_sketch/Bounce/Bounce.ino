

#include <AccelStepper.h>

#define fanPin 10
#define enStep 5

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 6, 7);

void setup()
{  
  // Change these to suit your stepper if you want
  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(550);
  stepper.moveTo(1600);
  digitalWrite(fanPin, HIGH);
  digitalWrite(enStep, HIGH);
}

void loop()
{
    // If at the end of travel go to the other end
    if (stepper.distanceToGo() == 0)
      stepper.moveTo(-stepper.currentPosition());
    stepper.run();
}
