
#include "DHT.h"

#define fanPin 10 // Arduino pin connected to relay which connected to fan
#define DHTPIN 12           // Arduino pin connected to relay which connected to DHT sensor
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

float temperature;    // temperature in Celsius
int control=35;
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 5000;           // interval at which to blink (milliseconds)

void setup()
{
  Serial.begin(9600); // initialize serial
  dht.begin();        // initialize the sensor
  pinMode(fanPin, OUTPUT); // initialize digital pin as an output
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
}

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) { //délai non bloquant
    previousMillis = currentMillis;
    temperature = dht.readTemperature();;  // lecture de la température par le capteur
    Serial.print("température : ");        // retour port série
    Serial.println(temperature);
    if (temperature<26) {                  //condition basse vitesse
      control=100;
      Serial.print("ventilateur :");
      Serial.print("40");
      Serial.println("%");
      analogWrite(fanPin, control);
    }
    if (temperature>=26){                   //accélération des ventilateurs jusqu'à 30°
      Serial.print("ventilateur :");
      Serial.println(temperature);
      if (temperature>=30) temperature=30;
      control=map(temperature, 26, 30, 100, 255);
      analogWrite(fanPin, control);
      control=map(control, 100, 255, 40, 100);
      Serial.print("ventilateur :");
      Serial.print(control);
      Serial.println("%");
      
    }
  }
}
