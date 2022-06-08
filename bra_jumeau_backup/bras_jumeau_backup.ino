#include <AccelStepper.h>
#include <Servo.h>
#include <Math.h>

int pos = 0;

Servo Rot; 
Servo Servo1; 
Servo Servo2; 
Servo Pince;

// input analogique
const int potpin0 = A0;  
int val0;
const int potpin1 = A1;  
int val1;
const int potpin2 = A2;  
int val2;
const int potpin3 = A3;  
int val3;

int domain0;
int domain1;
int sortie;

//input numérique (boutons)
const int bpPince = 2;
int bpPincestate = 0; 
int stepperdest = 0;
int stepperdestpast = 0;

AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

void setup() {
  // port série
  Serial.begin(9600);
  Serial.println("Initialisation");

  pinMode(bpPince, INPUT); // pin BP

  Rot.attach(5);        // pin servo
  Servo1.attach(6);     // pin servo
  Servo2.attach(7);     // pin servo
  Pince.attach(4);      // pin servo

  stepper.setSpeed(50); //vitesse pas-à-pas
  stepper.setMaxSpeed(1200);
  stepper.setAcceleration(500);
  stepper.moveTo(0);
  
  Rot.write(0);         //position initiale
  Servo1.write(40);     //position initiale
  Servo2.write(155);    //position initiale
  Pince.write(20);      //position initiale
}

void loop() {

  //lecture des pot
  val0 = analogRead(potpin0);            // lecture pot1
  if (val0 > 119) val0 = 119;
  val0 = map(val0, 0, 119, 20, 66);      // prod croix
  domain0 = map(val0, 40, 66, 100, 0);
  
  val1 = analogRead(potpin1);            // lecture pot1
  if (val1 > 165) val1 = 165;
  val1 = map(val1, 0, 165, 115, 160);    // prod croix
  domain1 = map(val1, 115, 160, 0, 100);
  sortie = domain0+domain1;
  val1 = map(sortie, 0, 100, 115, 160);
  Serial.println(val1); //debug print
  
  
  val2 = analogRead(potpin2);
  val2 = map(val2, 0, 673, 0, 180);     // prod croix 
  
  Servo1.write(val0);
  Servo2.write(val1);
  Rot.write(val2);

  //Pince
  if (digitalRead(bpPince) == 1) {
     Pince.write(90);
     } 
  else {
     Pince.write(50);
  }

  stepperdestpast=analogRead(potpin3);
  stepperdest=analogRead(potpin3);
  if (stepperdest > 640) stepperdest = 640;
  stepperdest = stepperdest*10.52;
 // Serial.println(stepperdest);
  
  stepper.moveTo(stepperdest);
  stepper.run();
}