#include <Servo.h>

Servo myservo;  // assignation servo

int potpin = A0;  // input analogique
int val;    // variable

void setup() {
  myservo.attach(9);  // pin servo
}

void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 0, 180);     // scale it for use with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  delay(15);                           // waits for the servo to get there
}
