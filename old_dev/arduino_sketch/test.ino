#include <Servo.h>  //arduino library
#include <math.h>   //standard c library

#define PI 3.141

Servo baseServo; 
Servo shoulderServo; 
Servo wristServo;

int command;

struct jointAngle{
  int base;
  int shoulder;
  int elbow;
};

struct jointAngle desiredAngle; //desired angles of the servos

//+++++++++++++++FUNCTION DECLARATIONS+++++++++++++++++++++++++++

void servoControl (int thePos, int theSpeed, Servo theServo);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




void setup()
{
  Serial.begin(9600);
  baseServo.attach(5);        // attaches the servo on pin 9 to the servo object
  shoulderServo.attach(6);
  wristServo.attach(7);

  Serial.setTimeout(50);      //ensures the the arduino does not read serial for too long
  Serial.println("started");
  baseServo.write(90);        //intial positions of servos
  shoulderServo.write(20);
  wristServo.write(120);
}

//primary arduino loop
void loop()
{
  while (Serial.available()){
    //read a serial command of the type (1,2,3d) where they are the joint angles for the position of the arm.

    desiredAngle.base = Serial.parseInt();
    desiredAngle.shoulder = Serial.parseInt();
    desiredAngle.elbow = Serial.parseInt();

    if(Serial.read() == 'd'){               // if the last byte is 'd' then stop reading and execute command 'd' stands for 'done'
        Serial.println("received signal");
            
    }

    //move the servo to the desired position
    servoControl(desiredAngle.base, 20, baseServo);
    servoControl(desiredAngle.shoulder, 20, shoulderServo);
    servoControl(desiredAngle.elbow, 20, wristServo);     

    Serial.println("done");
  } // end of serial while

}

//++++++++++++++++FUNCTION DEFITNITIONS++++++++++++++++++++++++++

void servoControl (int thePos, int theSpeed, Servo theServo){
  //thePos is the desired position the servo should be driven to
  // theSpeed is the delay between each increment of the servo position in milliseconds
  // theServo is the servo object that is to be controled.

  Serial.println("in servoControl");

  int startPos = theServo.read();        //read the current pos
  int newPos = startPos;

  //define where the pos is with respect to the command
  // if the current position is less that the actual move up
  if (startPos < thePos){
        while (newPos < (thePos - 5)){
          newPos = newPos + 5;
          theServo.write(newPos);
          delay(theSpeed);
        }   
  }

  else{
       while (newPos > (thePos + 5)){
         newPos = newPos - 5;
         theServo.write(newPos);
         delay(theSpeed);
       }   
  }
} // end of function