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

// paramètres et broches moteur pas-à-pas
#define MOTOR_STEPS 400
#define RPM 35
#define MICROSTEPS 1
#define DIR 6
#define STEP 7
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
const int stepper_enPin[] = {5};
const int stepper_dirPin[] = {6};
const int stepper_stepPin[] = {7};
//liste d'angles
double angle_current[] = {90, 90, 145};
double angle_next[] = {90, 90, 145};
//coordonnées du bras initialement, puis actuelles, puis suivantes
const double XYZ_base[] = {0, 0, -6, 1, 0, 0, 1}; //x,y,z, bool_move, bool_open, delay_to_next, type_of_action
double XYZ_current[] = {0, 0, -9, 1, 0, 1}; //x,y,z, bool_move, bool_open, delay_to_next, type_of_action
double XYZ_next[] = {0, 0, -9, 1, 0, 1}; //x,y,z, bool_move, bool_open, delay_to_next, type_of_action
//contraintes de translation (pas-à-pas)
const int stepper_delay[] = {100}; //27*22 for full step
const int stepper_maxsteps[] = {6340}; //max de pas
const double STEPS_PER_CM[] ={171}; //résultat calcul pas/CM
double stepper_correction[]={0};
// vérifier que l'angle du servo correpond bien à l'angle de la partie du bras commandée
// mettre le servo à 90°, puis mesurer l'angle réel
// la valeur de calibration est donc (90 - {réel angle par rapport à 90})
const double calibrate_TopArm=0;
const double calibrate_MiddleArm=0;
//compensation hauteur de la pince par rapport à l'extrémité du segment 2 du bras
const double calibrate_Z=8;
// communication
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;

void setup() {  
  // port série
  Serial.begin(9600);
  //envoi du mot start sur le port série
  Serial.println("start");
  //setup broches arduino
  pinMode(stepper_dirPin[0], OUTPUT);
  pinMode(stepper_stepPin[0], OUTPUT);
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, HIGH);
  int i = 0;
  for (i = 0; i < 3; i++) {
    pinMode(servo_Pin[i], OUTPUT);
  }
  // Set Coordinates a Base
  int y = 0;
  int z = 25;
  coordinate_move(0, y);
  for (i = 0; i < 3; i++) {
    servo[i].attach(servo_Pin[i], 500, 2500);
  }
  // initialisation capteur température
  dht.begin();
  //init pin EN
  stepper.setEnableActiveState(LOW);
  Serial.println("ready");
}

