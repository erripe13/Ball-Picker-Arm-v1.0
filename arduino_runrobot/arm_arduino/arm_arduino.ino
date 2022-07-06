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
#define ENDSTOP 11
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
//contraintes de translation (pas-à-pas)
//const int stepper_delay[] = {100}; //27*22 for full step
const int stepper_maxdeg[] = {1770}; //max de cm
const double DEG_PER_CM[] ={39,5}; //résultat calcul deg/CM
double stepper_correction[]={0};
double step_deg_remain = 45;
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
  //pinMode(stepper_dirPin[0], OUTPUT);
  //pinMode(stepper_stepPin[0], OUTPUT);
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
  Serial.println("calib go");
  calib_x();
  Serial.println("stepper test go");
  test_stepper();
  Serial.println("ready");
}

void loop() {
  coordinate_move(20, 20);
  // recvWithStartEndMarkers();
  // showNewData();
  // if (newData==true && loop==true) {
  //   coordinate_move(XYZ_next[0],XYZ_next[1]);
  //   newData=false;
  //   Serial.println("done");
  // }
}

void coordinate_move(double xEnd, double yEnd) {
  double zEnd = 25.0;
  double xStart = XYZ_current[0];
  double yStart = XYZ_current[1];
  double zStart = XYZ_current[2];

  //application de la calibration de pas du moteur
  double x_to_steps = DEG_PER_CM[0];

  //mouvement en y ?
  double zDelta = zEnd - zStart;
  //mouvement en z ?
  double xDelta = xEnd - xStart;

  double x_stepper_steps = x_to_steps * abs(xDelta);

  if (xDelta != 0) {
    if (xDelta > 0) {
      stepper_advance(x_stepper_steps, 0);
    } else {
      stepper_advance(x_stepper_steps, 1);
    }
  }

    if (zDelta < 0) {
      //le bras va descendre
      // bouge en Y
      get_angles_from_yz(yEnd, zStart);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);

      // bouge en Z
      get_angles_from_yz(yEnd, zEnd);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);

    } else {
      //le bras va monter
      get_angles_from_yz(yStart, zEnd);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);
      get_angles_from_yz(yEnd, zEnd);
      twoarm_step_coordinate(angle_next[0], angle_next[1]);
    }



  //prints de débug :
//  Serial.println(" //////// ");
//  Serial.print(" xStart=  "); Serial.print(xStart); Serial.print(" yStart=  "); Serial.println(yStart);
//  Serial.print("Angle Top Arm="); Serial.print(angle_TopArm); Serial.print(" Angle Middle Arm=  "); Serial.println(angle_MiddleArm);
//  Serial.print("Angle Top Arm_next="); Serial.print(angle_TopArm_next); Serial.print(" Angle Middle Arm_next=  "); Serial.println(angle_MiddleArm_next);
//  Serial.print(" xEnd=  "); Serial.print(xEnd);   Serial.print(" yEnd=  "); Serial.println(yEnd);

  XYZ_current[0] = xEnd;
  XYZ_current[1] = yEnd;
  XYZ_current[2] = zEnd;
  //XYZ_current[3] = liftgrab_motion;
}

void stepper_advance(double steps, int dir) {
  stepper.enable();
  // génération de la pwm pour le driver TMC2208
  // vérification si besoin de compensation pas-à-pas
  // if (abs(stepper_correction[stepper_num]) > 1) {
  //   if (stepper_correction[stepper_num]>1){
  //     //ajout d'un pas si la compensation >1
  //     steps++;
  //     stepper_correction[stepper_num]--;
  //   } else {
  //     steps--;
  //     stepper_correction[stepper_num]++;
  //   }
  // }

  // paramètre de direction de translation
  if (dir == 0) {
    stepper.rotate(steps);
    //digitalWrite(stepper_dirPin[stepper_num], HIGH);
  } else {
    stepper.rotate(-steps);
    //digitalWrite(stepper_dirPin[stepper_num], LOW);
  }

  // envoi pwm driver TMC2208
  // while (1) {
  //   digitalWrite(stepper_stepPin[stepper_num], HIGH);
  //   delayMicroseconds(stepper_delay[stepper_num]);
  //   digitalWrite(stepper_stepPin[stepper_num], LOW);
  //   delayMicroseconds(stepper_delay[stepper_num]);

  //   steps--;
  //   if (steps < 1) break;
  // }


  // stockage du nombre de pas corrigés pour tenir les comptes et ne pas décalibrer
  // if (steps > 0 && steps <1) {
  //   if (dir ==0) {
  //     stepper_correction[stepper_num]+=steps;
  //   } else {
  //     stepper_correction[stepper_num]-=steps;
  //   }
  // }
  
  // Serial.print("reste de pas stocké");
  // Serial.println(stepper_correction[stepper_num]);
  stepper.disable();
}

void calib_x(){
  stepper.enable();
  stepper.startMove(-2500);
  if (digitalRead(ENDSTOP) == HIGH){
    stepper.stop();
    XYZ_current[0]=0;
  }
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
void twoarm_step_coordinate(double toparm_target, double middlearm_target) {

  double incr_steps0=1;
  double incr_steps1= 1;
  int inner_step_delay0 = 0;
  int inner_step_delay1 = 0;
  int outer_step_delay = 30;
  double i, j;
  int e0 = 0;
  int e1 = 0;

  //détermine quel segment a le meilleur delta en terme d'efficacité de mouvement
  double delta0 = abs(angle_current[0] - toparm_target);
  double delta1 = abs(angle_current[1] - middlearm_target);

  //coordination des deux servos avec gestion de la vitesse pour que les deux segments finissent leurs déplacements en même temps

  if (delta0!=0 && delta1!=0) {
    if (delta0 >= delta1) {
      incr_steps0 = (delta0 / delta1)*incr_steps1;
      //au plus le déplacement est grand au plus on ralentit
      inner_step_delay0=(delta0/delta1)*0.5;
   
      outer_step_delay=outer_step_delay-inner_step_delay0;
    } else {
      incr_steps1 = (delta1 / delta0)*incr_steps0;
      inner_step_delay1=(delta1/delta0)*0.5;

      outer_step_delay=outer_step_delay-inner_step_delay1;
    }
  }
  
  if (outer_step_delay<0) {
    outer_step_delay=0;
  }
  
  //identification de la direction des pas "soft" des servos
  if (angle_current[0] > toparm_target) {
    i = -incr_steps0;
  } else {
    i = incr_steps0;
  }
  if (angle_current[1] > middlearm_target) {
    j = -incr_steps1;
  } else {
    j = incr_steps1;
  }
  
  // contrôle général des servos
  while (1) {
    // segment1
    if (abs(angle_current[0] - toparm_target) > incr_steps0) {
      servo_steps(0, angle_current[0] + i, incr_steps0, inner_step_delay0);
    } else {
      servo_steps(0, toparm_target, incr_steps0, inner_step_delay0);
      e0 = 1;
    }
    // segment2
    if (abs(angle_current[1] - middlearm_target) > incr_steps1) {
      servo_steps(1, angle_current[1] + j, incr_steps1, inner_step_delay1);
    } else {
      servo_steps(1, middlearm_target, incr_steps1, inner_step_delay1);
      e1 = 1;
    }
    delay(outer_step_delay);
    if ((e0 + e1) >= 2) break;

  }

  //Serial.println("--> two arm step End /");

}
void get_angles_from_yz(double y, double z) {

  //refer to trigonometry illustration for variable description

  double H, s1, s2, aB, aA, aQ, servo1angle, servo2angle, y2, z2, y3, z3;

  //arm length in cm
  int L = 13;

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
  angle_next[0] = angle_next[0] + calibrate_TopArm;
  angle_next[1] = angle_next[1] + calibrate_MiddleArm;

}


void servo_Open(bool openVal) {

  int servo_num = 2;
  int open_angle = servoGrip_val[0];
  int close_angle = servoGrip_val[1];
  if (openVal == true) {
    servo_steps(servo_num, open_angle, 1, 5);
  } else {
    servo_steps(servo_num, close_angle, 1, 5);
  }
  
  XYZ_current[4] = openVal;
}
void test_stepper() {
  stepper_advance(stepper_maxdeg[0], 0);
  delay(1000);
  stepper_advance(stepper_maxdeg[0], 1);
  delay(1000);
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
void test_servo_home(int servo_num) {
  
  int angle_default = 0;

  //ref servo_steps(int servo_num, int angle_target, int incr_step=10, int step_delay=50)

  //segment1
  if (servo_num == 0) {
    angle_default = 10;
  }
  //segment2
  if (servo_num == 1) {
    angle_default = 90;
  }
  //servo pince
  if (servo_num == 2) {
    angle_default = servoGrip_val[2];
  }

  servo_steps(servo_num, angle_default);

}
void test_getangles(double y, double z) {
  get_angles_from_yz(y,z);
  Serial.print("Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);
  Serial.print("servo1: ");
  Serial.print(angle_next[0]-calibrate_TopArm);
  Serial.print(" servo2: ");
  Serial.println(angle_next[1]-calibrate_MiddleArm);
  Serial.print("servo1 calibrated: ");
  Serial.print(angle_next[0]);
  Serial.print(" servo2 calibrated: ");
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
        Serial.println(receivedChars);
        parseData();

        bool printmsg=false;
        if (printmsg == true) {
          Serial.print("X: ");
          Serial.print(XYZ_next[0]);
          Serial.print(" Y: ");
          Serial.print(XYZ_next[1]);
          Serial.print(" Z: ");
          Serial.print(XYZ_next[2]);
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
  XYZ_next[0] = atof(strtokIndx);     // conversion en float
  //grab Y
  strtokIndx = strtok(NULL, ",");
  XYZ_next[1] = atof(strtokIndx);     // conversion en float
  //grab Z
  strtokIndx = strtok(NULL, ",");
  XYZ_next[2] = atof(strtokIndx);     // conversion en float
}
