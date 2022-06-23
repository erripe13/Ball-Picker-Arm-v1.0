#include "DHT.h"

#define fanPin 10 // Arduino pin connected to relay which connected to fan
#define DHTPIN 12           // Arduino pin connected to relay which connected to DHT sensor
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

float temperature;    // temperature in Celsius
int control=35;
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)

void setup()
{
  Serial.begin(9600); // initialize serial
  dht.begin();        // initialize the sensor
  pinMode(fanPin, OUTPUT); // initialize digital pin as an output
}

void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    temperature = dht.readTemperature();;  // read temperature in Celsius
    Serial.print("température : ");
    Serial.println(temperature);
    if (temperature<26) {
      control=100;
      Serial.print("ventilateur: ");
      Serial.print("40");
      Serial.println("%");
      analogWrite(fanPin, control);
    }
    if (temperature>=26){
      Serial.print("température : ");
      Serial.println(temperature);
      if (temperature>=30) temperature=30;
      control=map(temperature, 26, 30, 100, 255);
      analogWrite(fanPin, control);
      control=map(control, 0, 255, 0, 100);
      Serial.print("ventilateur: ");
      Serial.print(control);
      Serial.println("%");
      
    }
  }
}
