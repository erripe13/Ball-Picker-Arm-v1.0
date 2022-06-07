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
int buth = 115;
int butb = 166;
int angle;

//input numérique (boutons)
const int bpPince = 2;
int bpPincestate = 0;
int stepperdest = 0;

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

  //stepper.setSpeed(50); //vitesse pas-à-pas
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(900);
  stepper.moveTo(0);

  Rot.write(0);         //position initiale
  Servo1.write(40);     //position initiale
  Servo2.write(155);    //position initiale
  Pince.write(20);      //position initiale
}

void loop() {

  //contrôles du bras
  val0 = analogRead(potpin0);            // lecture pot1
  if (val0 > 119) val0 = 119;
  val0 = map(val0, 0, 119, 28, 66);      // prod croix
  domain0 = map(val0, 28, 66, 100, 0);
  val1 = analogRead(potpin1);            // lecture pot2
  if (val1 > 165) val1 = 165;
  val1 = map(val1, 0, 165, 115, 160);    // prod croix
  domain1 = map(val1, 115, 160, 0, 100);
  sortie = domain0 + domain1;
  val1 = map(sortie, 0, 100, buth, butb);
  Serial.println(val1); //debug print

  //Pince
  if (digitalRead(bpPince) == 1) {
    Pince.write(90);
  }
  else {
    Pince.write(50);
  }

  //translation
  stepperdest = analogRead(potpin3);
  if (stepperdest > 640) stepperdest = 640; //butée pour éviter l'instabilité de fin de potentiomètre
  stepperdest = stepperdest * 10.52; //640 unités potentiomètre en 40cm
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(900);
  stepper.moveTo(stepperdest); //assignation destination stepper

  //rotation si translation pas trop avancée pour éviter chocs dans la cage
  if (stepperdest < 3000) {
    val2 = analogRead(potpin2);
    if (val2 > 670) val2 = 673;
    if (val2 < 4) val2 = 0;
    val2 = map(val2, 0, 673, 0, 180);     // prod croix
  }
  else {
    if (val2 >= 90) val2 = 180;
    else val2 = 0;
  }
  //exécution des moteurs en résultat de tous les calculs
  Servo1.write(val0);
  Servo2.write(val1);
  slowRot(val2);
  stepper.run();

}

void slowRot(int i) {
  angle = Rot.read();
  if ( i >= angle) {
    for (angle = angle; angle <= i; angle = angle + 1) { 
      Rot.write(angle);
      delay(10);
    }
  }
  else
  { for (angle = angle; angle >= i; angle = angle - 1){ 
      Rot.write(angle);
      delay(10);
    }
  }
}
