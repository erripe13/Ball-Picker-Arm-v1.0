// par Pierre Mirouze
// FabriqExpo Exploradôme de Vitry
// programme exécuté par l'Arduino contrôlant les moteurs du bras


#include <Arduino.h>
#include "DHT.h"
#include <Servo.h>
#include <math.h>
#include "BasicStepperDriver.h"

// paramètres et broches pour gestion thermique
#define fanPin 10
#define DHTPIN 12
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float temperature;
int control=35;

// paramètres et broches moteur
#define STOPPER_PIN 11
#define MOTOR_STEPS 400
#define RPM 45
#define MICROSTEPS 1
#define enPin 5
#define DIR 6
#define STEP 7
#define SLEEP 5
#define cutpower 8
#define cutpower2 9
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);


// paramètres et broches du robot
//initialisation de la liste des servos
Servo servo[3];
// setup broches utilisées
const int servo_Pin[] = {2, 3, 4}; //segment1,segment2,pince
//définition des valeurs de serrage/désserrage sous le format suivant :
//{ouvert-max, fermé-min, repos}
const int servoGrip_val[]= {114,70,85};  
//broches moteur pas-à-pas
//const int stepper_enPin[] = {5};
//const int stepper_dirPin[] = {6};
//const int stepper_stepPin[] = {7};
//liste d'angles
double angle_current[] = {90, 90, 145};
double angle_next[] = {90, 90, 145};
//coordonnées du bras initialement, puis actuelles, puis suivantes
const double XYZ_base[] = {0, 0, -40}; //x,y,z, bool_move, bool_open, delay_to_next, type_of_action
double XYZ_current[] = {0, 0, -40}; //x,y,z, bool_move, bool_open, delay_to_next, type_of_action
double XYZ_next[] = {0, 0, -9}; //x,y,z, bool_move, bool_open, delay_to_next, type_of_action
// vérifier que l'angle du servo correpond bien à l'angle de la partie du bras commandée
// mettre le servo à 90°, puis mesurer l'angle réel
// la valeur de calibration est donc (90 - {réel angle par rapport à 90})
const double calibrate_TopArm=0;
const double calibrate_MiddleArm=0;
//compensation hauteur de la pince par rapport à l'extrémité du segment 2 du bras
const double calibrate_Z=8;
//étalo step
const double DEG_PER_CM=38.8;
// communication
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
double degmove;
double xdest, ydest;
double servo1angle, servo2angle;

void setup() {  
  // port série
  Serial.begin(9600);
  //envoi du mot start sur le port série
  Serial.println("start");
  //setup broches arduino
  pinMode(cutpower, OUTPUT);
  pinMode(cutpower2, OUTPUT);
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, HIGH);
  //pinMode(enPin, OUTPUT);
  pinMode(STOPPER_PIN, INPUT);
  //digitalWrite(enPin, LOW);

  int i = 0;
  for (i = 0; i < 3; i++) {
    pinMode(servo_Pin[i], OUTPUT);
  }
  // Set Coordinates a Base
  int y = 0;
  int z = -40  ;
  //coordinate_move(0, y);
  for (i = 0; i < 3; i++) {
    servo[i].attach(servo_Pin[i], 500, 2500);
  }
  // initialisation capteur température
  dht.begin();

  stepper.begin(RPM, MICROSTEPS);
  //stepper.setEnableActiveState(LOW);
  pinMode(STOPPER_PIN, INPUT);
  
  calib_x();
  Serial.println("ready");
//  digitalWrite(cutpower, HIGH);
//  servo[0].write(90);
//  servo[1].write(90);
//  delay(6000);
//  digitalWrite(cutpower, LOW);
  //get_angles_from_yz(30, -35.35);
  //xmove(20.0);
}

void loop() {
bool loop=true;
recvWithStartEndMarkers();
showNewData();
  if (newData==true && loop==true) {
      xymove(xdest, ydest);
      newData=false;
      Serial.println("done");
  }
}

void calib_x(){
  //digitalWrite(enPin, LOW);
  Serial.println("stepper calib");
  while (digitalRead(STOPPER_PIN) == LOW){
      stepper.rotate(-15);
  }
  Serial.println("ENDSTOP");
  stepper.stop();
  stepper.rotate(1750);
  stepper.stop();
  delay(2000);
  //digitalWrite(enPin, HIGH);
}



void servo_steps(int servo_num, double angle_target, double incr_step = 10, int step_delay = 100) {
  // cette commande permet d'envoyer les instructions par paquets de 25 degrés. utile pour des servos bas de gamme.

  int set_angle;
  int angle_start = angle_current[servo_num];

  if (angle_start > angle_target) {
    //commencer à partir de l'angle_start, puis déplacer le servo par incr_pas dans l'angle_target

    for (set_angle = angle_start; set_angle >= angle_target; set_angle -= incr_step) {
      servo[servo_num].write(set_angle);
      //Serial.print("angle servo:");
      //Serial.println(set_angle);
      delay(step_delay);
    }
  } else {

    for (set_angle = angle_start; set_angle <= angle_target; set_angle += incr_step) {
      servo[servo_num].write(set_angle);
      //Serial.print("angle servo:");
      //Serial.println(set_angle);
      delay(step_delay);
    }
  }

  // assurer l'instruction d'angle
  servo[servo_num].write(angle_target);
  //mettre à jour la donnée d'angle dans le registre de position
  angle_current[servo_num] = angle_target;

}

void get_angles_from_yz(double y, double z) {
  Serial.print("y :");
  Serial.println(y);
  Serial.print("z :");
  Serial.println(z);
  //refer to trigonometry illustration for variable description
  double H, s1, s2, aB, aA, aQ, y2, z2, y3, z3;
  //arm length in cm
  int L = 25;
  H= sqrt (pow(y,2) + pow(z,2));
  s1=H/2;
  s2=sqrt (pow(L,2) - pow(s1,2));
  aB=atan(s2/s1);
  y2=y/2;
  z2=z/2;
  aA=atan(y2/z2);
  servo1angle=aA+aB;
  servo1angle= (servo1angle/ (2 * M_PI)) * 360;
  y3 = -L*sin(aA+aB);
  z3 = -L* cos(aA+aB);
  servo2angle=atan((y-y3)/(z-z3));  
  servo2angle= (servo2angle / (2 * M_PI)) * 360;
  //tangent calculation changes when servo2 exceeds 90 degrees, correction below
  if ((z-z3)>0) {
    servo2angle=servo2angle-180;
  }
  
  servo2angle=abs(servo2angle);
  Serial.print("Angle 1 bras :");
  Serial.println(servo1angle);
  Serial.print("Angle 2 bras :");
  Serial.println(servo2angle);
  servo1angle=map(servo1angle, 0, 90, 90, 0);
  servo2angle=map(servo2angle, 0, 90, 180, 90);
  //conversion degrés bras théorique en angles servo étalonnés
  //servo1angle = 90-servo1angle;
  Serial.print("Angle 1 servo :");
  Serial.println(servo1angle);
  //servo2angle = -servo2angle;
  Serial.print("Angle 2 servo:");
  Serial.println(servo2angle);
  
  //envoi des angles
  servo[0].write(servo1angle);
  servo[1].write(servo2angle);
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
  if (Serial.available() > 0) {
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // fin du string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
   
}
}

void showNewData() {
    if (newData == true) {
        //Serial.println(receivedChars);
        parseData();

        bool printmsg=true;
        if (printmsg == true) {
          Serial.print("X: ");
          Serial.println(xdest);
          Serial.print(" Y: ");
          Serial.println(ydest);
        }
        //newData = false;
    }
}

void parseData() {

  // formattage des données reçues

  //format : <x,y,z> = <23,56,89> {17}
  //X: 7.00 Y: 8.00 Z: 9.00 bool_move: 1.00 bool_open: 0.00 delay_ms: 10.00 move_type: 1.00

  char * strtokIndx;
 
  //grab X
  strtokIndx = strtok(receivedChars,","); 
  xdest = atof(strtokIndx);     // conversion en float
  //grab Y
  strtokIndx = strtok(NULL, ",");
  ydest = atof(strtokIndx);     // conversion en float

}

void xymove(double degmove, double ymove){
  ymove=ymove-30.0; //correction 
  ymove=-ymove;
  servo[0].write(20);
  servo[1].write(110);
  servo[2].write(70);
  digitalWrite(cutpower, HIGH);
  digitalWrite(cutpower2, HIGH);
  Serial.print("GO X deg : ");
  degmove=45.5-degmove;
  degmove=degmove+8.0;
  if (degmove>=45) degmove=45;
  Serial.println(degmove);
  degmove=degmove*38.8;
  //degmove=abs(degmove);
  //digitalWrite(enPin, LOW);
  Serial.println(degmove);
  stepper.rotate(-degmove);
  stepper.stop();
  delay(2000);
  get_angles_from_yz(ymove, -26.0);
  delay(500);
  servo[2].write(20);
  delay(3000);
  servo[0].write(20);
  servo[1].write(110);
  stepper.rotate(degmove);
  stepper.stop();
  delay(1000);
  servo[2].write(70);
  delay(1000);
  digitalWrite(cutpower, LOW);
  digitalWrite(cutpower2, LOW);
  delay(4000);
  //digitalWrite(enPin, HIGH);
}
