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

void xymove(double degmove, double ymove){
  digitalWrite(cutpower, HIGH);
  digitalWrite(cutpower2, HIGH);
  servo_steps(0, 20)
  Serial.print("GO X deg : ");
  degmove=45.5-degmove;
  degmove=degmove+8.0;
  Serial.println(degmove);
  degmove=degmove*38.8;
  //degmove=abs(degmove);
  //digitalWrite(enPin, LOW);
  Serial.println(degmove);
  stepper.rotate(-degmove);
  stepper.stop();
  delay(2000);
  servo_steps(0, 30)
  stepper.rotate(degmove);
  servo_steps(0, 20)
  stepper.stop();
  digitalWrite(cutpower, LOW);
  digitalWrite(cutpower2, LOW);
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

  //refer to trigonometry illustration for variable description

  double H, s1, s2, aB, aA, aQ, servo1angle, servo2angle, y2, z2, y3, z3;

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

  //matrix multiplication - counterclockwise rotation
  y3 = -L*sin(aA+aB);
  z3 = -L* cos(aA+aB);

  servo2angle=atan((y-y3)/(z-z3));  

  servo2angle= (servo2angle / (2 * M_PI)) * 360;
  //tangent calculation changes when servo2 exceeds 90 degrees, correction below
  if ((z-z3)>0) {
    servo2angle=servo2angle-180;
  }

  //Absolute Top Arm Angle
  //Top Arm moves 0 to +90
  angle_next[0] = servo1angle;

  //Absolute Middle Arm Angle
  //Midle Arm moves 0 to +90
  angle_next[1] = -servo2angle;


  //Convert to SERVO Angle
  //in this case, a 90 servo position is equal to 71 degrees for Top arm
  //90 servo position is equal to 65 Middle Arm
  // angle_next[0] = angle_next[0] + calibrate_TopArm;
  // angle_next[1] = angle_next[1] + calibrate_MiddleArm;

}

void test_servo(int servo_num) {

  int angle_max = 0;
  int angle_min = 0;
  int angle_default = 0;

  //ref servo_steps(int servo_num, int angle_target, int incr_step=10, int step_delay=50)

  //segment1
  if (servo_num == 0) {
    //Butées servo1 réelles
    angle_max = 80;
    angle_min = 20;
    angle_default = 46;
  }
  //segment2
  if (servo_num == 1) {
    //Butées servo2 réelles
    angle_max = 165;
    angle_min = 140;
    angle_default = 155;
  }
  //Butées servo pince réelles
  if (servo_num == 2) {
    angle_max = servoGrip_val[0];
    angle_min = servoGrip_val[1];
    angle_default = servoGrip_val[2];
  }

  servo_steps(servo_num, angle_max);
  delay(1000);
  servo_steps(servo_num, angle_min);
  delay(1000);
  servo_steps(servo_num, angle_default);

}

void test_getangles(double y, double z) {
  get_angles_from_yz(y,z);
  Serial.print("Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);
  Serial.print("servo1: ");
  Serial.print(angle_next[0]);
  Serial.print(" servo2: ");
  Serial.println(angle_next[1]);
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
