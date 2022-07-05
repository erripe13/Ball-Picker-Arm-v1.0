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

int i=1;
int pos = 0;    // variable to store the servo position

unsigned long previousMillis = 0;
const long interval = 15;           // interval 

void setup() {
  servo1.attach(2); 
  servo2.attach(3);
  pince.attach(4);
  dht.begin();   
  pinMode(en6V, OUTPUT);
  pinMode(en5V, OUTPUT);
  Serial.begin(9600); // initialize serial
  pinMode(fanPin, OUTPUT); // initialize digital pin as an output
  pince.write(70);
}

void loop() {

  temperature = dht.readTemperature();  // read temperature in Celsius
  Serial.print("température : ");
  Serial.println(temperature);
  digitalWrite(en6V, LOW);
  digitalWrite(en5V, LOW);
  digitalWrite(fanPin, HIGH);

  servo2.write(90);
  //picktest();
}
void picktest(){
  for(i=1; i<=2; i++){
    
  servo1.write(60);
  servo2.write(100);
  delay(500);
  
  servo2.write(110);
  delay(500);
  
  pince.write(25);
  delay(1000);  
  
  servo1.write(40);
  servo2.write(90);
  delay(2000);

  servo1.write(30);
  servo2.write(120);
  delay(500);
  
  servo2.write(135);
  delay(500);
  
  pince.write(70);
  delay(1000);

  servo2.write(120);
  delay(500);

  servo1.write(40);
  servo2.write(90);
  delay(2000);

  servo1.write(30);
  servo2.write(120);
  delay(500);
  
  servo2.write(135);
  delay(500);

  pince.write(25);
  delay(1000);  

  servo1.write(40);
  servo2.write(90);
  delay(2000);

  servo1.write(60);
  servo2.write(100);
  delay(500);
  
  servo2.write(110);
  delay(500);
  
  pince.write(70);
  delay(1000);  

  servo2.write(100);
  delay(500);
  
  servo1.write(40);
  servo2.write(90);
  delay(2000);

  }
  
  digitalWrite(en6V, LOW);
  digitalWrite(en5V, LOW);
  delay(15000);
}
