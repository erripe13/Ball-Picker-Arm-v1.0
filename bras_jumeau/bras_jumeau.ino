#include <AccelStepperWithDistance.h>
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

//input numérique (boutons)
const int bpPince = 2;
int bpPincestate = 0; 
const int bprotG = 10;
int bprotGstate = 0; 
const int bprotD = 11;
int bprotDstate = 0; 
const int bpAV = 12;
int bpAVstate = 0; 
const int bpAR = 13;
int bpARstate = 0; 

AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

void setup() {
  // port série
  Serial.begin(9600);
  Serial.println("Initialisation");

  pinMode(bprotG, INPUT);  // pin BP
  pinMode(bprotD, INPUT);  // pin BP
  pinMode(bpAV, INPUT);    // pin BP
  pinMode(bpAR, INPUT);    // pin BP
  pinMode(bpPince, INPUT); // pin BP


  Rot.attach(5);     // pin servo
  Servo1.attach(6);  // pin servo
  Servo2.attach(7);  // pin servo
  Pince.attach(4);   // pin servo

  stepper.setSpeed(50); //vitesse pas-à-pas
  
  Rot.write(90);     //position initiale
  Servo1.write(49);  //position initiale
  Servo1.write(144); //position initiale
  Pince.write(90);   //position initiale
}

void loop() {
  //lecture des BP
  bpPincestate = digitalRead(bpPince);
  bprotGstate = digitalRead(bprotG);
  bprotDstate = digitalRead(bprotD);
  bpAVstate = digitalRead(bpAV);
  bpARstate = digitalRead(bpAR);
  
  //lecture des pot
  val0 = analogRead(potpin0);            // lecture pot1
  //val0 = map(val0, 0, 1023, 20, 75);     // prod croix
  val1 = analogRead(potpin1);            // lecture pot1
  //val1 = map(val1, 0, 1023, 140, 165);     // prod croix

  //écriture des angles
  Serial.println(val0); //debug print
  Serial.println(val1); //debug print
  //Servo1.write(val0);
  //Servo2.write(val1);

  //Pince
  if (bpPincestate == HIGH) {
     Pince.write(70);
     } 
  else {
     Pince.write(140);
  }

  //Rotation
  rotationControl();
  //Translation
  translationControl();
  
  delay(1);
}

void rotationControl(void){
  if (bprotGstate == HIGH) {
    pos++;  
    delay(2);
    Pince.write(pos);
  }
  else if (bprotDstate == HIGH) {
    pos--; 
    delay(2);
    Pince.write(pos);
  }
}

void translationControl(void){
}
