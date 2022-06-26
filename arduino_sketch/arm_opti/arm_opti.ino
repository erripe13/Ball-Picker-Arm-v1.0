#include "DHT.h"
#include <Servo.h>
#include <math.h>

#define fanPin 10 // Arduino pin connected to relay which connected to fan
#define DHTPIN 12           // Arduino pin connected to relay which connected to DHT sensor
#define DHTTYPE DHT11
#define en6V 8
#define en5V 9

DHT dht(DHTPIN, DHTTYPE);
float temperature;    // temperature in Celsius
int control=35;
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 2000;           // intervmpal at which to blink (milliseconds)

//initialisation de la liste des servos
Servo servo1;
Servo servo2;
Servo pince;                         
// setup broches utilisées
//const int servo_Pin[] = {2, 3, 4}; //segment1,segment2,pince
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
const int stepper_delay[] = {27 * 28}; //27*22 for full step
const int stepper_maxsteps[] = {6340}; //max de pas
const double STEPS_PER_CM[] ={171}; //résultat calcul pas/CM
double stepper_correction[]={0};
// vérifier que l'angle du servo correpond bien à l'angle de la partie du bras commandée
// mettre le servo à 90°, puis mesurer l'angle réel
// la valeur de calibration vaut donc (90 - {réel angle par rapport à 90})
const double calibrate_TopArm=0;
const double calibrate_MiddleArm=-90;
//compensation hauteur de la pince par rapport à l'extrémité du segment 2 du bras
const double calibrate_Z=8;
// communication
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;


void setup() {
    // port série
    Serial.begin(9600);
    //envoi du start sur le port série
    Serial.println("Démarrage de l'initialisation");

    //setup pin
    pinMode(stepper_dirPin[0], OUTPUT);
    pinMode(stepper_stepPin[0], OUTPUT);
    pinMode(stepper_enPin[0], OUTPUT);
//  pinMode(servo_Pin[0], OUTPUT);
//  pinMode(servo_Pin[1], OUTPUT);
//  pinMode(servo_Pin[2], OUTPUT);
    servo1.attach(2);
    servo2.attach(3);
    pince.attach(4);
    pinMode(en6V, OUTPUT);
    pinMode(en5V, OUTPUT);            
    pinMode(fanPin, OUTPUT); 
    dht.begin();
}

void loop() {
    temperature = dht.readTemperature();;  // read temperature in Celsius
    Serial.print("température : ");
    Serial.println(temperature);
    digitalWrite(en6V, LOW);
    digitalWrite(en5V, LOW);
    digitalWrite(fanPin, HIGH);
    
}
