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
  stepper.setSpeed(800);
  stepper.setMaxSpeed(150);
  stepper.setAcceleration(100);

  //Rot.write(0);         //position initiale
  Servo1.write(40);     //position initiale
  Servo2.write(155);    //position initiale
  slowRot(0);
  Pince.write(40);      //position initiale
}

void loop() {
  pantin();
  //stackunstack();
}

void slowRot(int i) {
  angle = Rot.read();
  if ( i >= angle) {
    for (angle = angle; angle <= i; angle = angle + 1) { 
      Rot.write(angle);
      delay(13);
    }
  }
  else
  { for (angle = angle; angle >= i; angle = angle - 1){ 
      Rot.write(angle);
      delay(13);
    }
  }
}

void stackunstack(){
  slowRot(0);
  Servo1.write(50);     //position initiale
  Servo2.write(90);    //position initiale
  delay(1000);
  Servo1.write(50);     //position initiale
  Servo2.write(90);    //position initiale
  delay(1000);
  Servo1.write(50);     //position initiale
  Servo2.write(90);    //position initiale
  delay(1000);
}

void pantin() {

  //contrôles du bras
  val0 = analogRead(potpin0);            // lecture pot1
  if (val0 > 130) val0 = 130;
  val0 = map(val0, 0, 130, 28, 76);      // prod croix
  domain0 = map(val0, 28, 76, 100, 0);
  
  val1 = analogRead(potpin1);            // lecture pot2
  if (val1 > 160) val1 = 160;
  val1 = map(val1, 0, 165, 90, 160);    // prod croix
  domain1 = map(val1, 90, 160, 0, 100);
  sortie = domain0 + domain1;
  val1 = map(sortie, 0, 100, buth, butb);
  Serial.print("s1="); //debug print
  Serial.println(val1);
  Serial.println("s2="); //debug print
  Serial.println(val2);
  
  
  //Pince
  if (digitalRead(bpPince) == 1) {
    Pince.write(80);
  }
  else {
    Pince.write(60);
  }

  //translation
//  stepperdest = analogRead(potpin3);
//  if (stepperdest > 640) stepperdest = 640; //butée pour éviter l'instabilité de fin de potentiomètre
//  stepperdest = stepperdest * 10.52; //640 unités potentiomètre en 40cm
//  stepper.moveTo(stepperdest); //assignation destination stepper
//  stepper.run();
  //rotation si translation pas trop avancée pour éviter chocs dans la cage
  //if (stepper.currentPosition() < 3000) {
  val2 = analogRead(potpin2);
  Serial.println(val2);
  if (val2 > 670) val2 = 675;
  if (val2 < 4) val2 = 0;
  val2 = map(val2, 0, 675, 0, 170);     // prod croix
  //}
  //else {
  //  if (val2 >= 90) val2 = 180;
  //  else val2 = 0;        
  //}
  //exécution des moteurs en résultat de tous les calculs
  Servo1.write(val0);
  Servo2.write(val1);
  slowRot(val2);
}
