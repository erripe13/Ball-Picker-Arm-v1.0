#include "DHT.h"
#include <Servo.h>

#define fanPin 10 // Arduino pin connected to relay which connected to fan
#define en6V 8
#define en5V 9
#define DHTPIN 12           // Arduino pin connected to relay which connected to DHT sensor
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

float temperature;    // temperature in Celsius
int control=110;

Servo servo1;
Servo servo2;
Servo pince;

int pos = 0;    // variable to store the servo position

unsigned long previousMillis = 0;
const long interval = 15;           // interval 

void setup() {
  servo1.attach(2);  // attaches the servo on pin 9 to the servo object
  servo2.attach(3);
  pince.attach(4);
  dht.begin();   
  pinMode(en6V, OUTPUT);
  pinMode(en5V, OUTPUT);
  Serial.begin(9600); // initialize serial
  pinMode(fanPin, OUTPUT); // initialize digital pin as an output
}
void loop() {

  temperature = dht.readTemperature();;  // read temperature in Celsius
  Serial.print("température : ");
  Serial.println(temperature);
  digitalWrite(en6V, LOW);
  digitalWrite(en5V, LOW);
  digitalWrite(fanPin, HIGH);
  
  servo1.write(15);              // tell servo to go to position in variable 'pos'
  servo2.write(180);              // tell servo to go to position in variable 'pos'

  delay(10);
}
