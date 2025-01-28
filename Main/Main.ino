#include "LEDController.h"
#include "MotorController.h"
#include "SensorManager.h"
#include "Communication.h"

int timer = 1000;
int ledPin = 13;
SensorManager sensorManager;
Commun communicationManager;
LEDController ledCon;

void setup() {
  Serial.begin(115200);
  ledCon = LEDController(7);
  sensorManager = SensorManager(6);
  communicationManager = Commun();
}

void loop() {
  ledCon.changeState();
  delay(1000);
}