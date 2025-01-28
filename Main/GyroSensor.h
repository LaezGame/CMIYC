#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

class GyroSensor{
  public:
    String address;
    //Adafruit_BNO055 bno;;
    

  GyroSensor(){
    this->address = "x28";
    this->startGyro();
  }

  GyroSensor(String newAddress){
    this->address = newAddress;
    this->startGyro();
  }

  void startGyro(){
  /*this->bno = Adafruit_BNO055(55, this->address);
  while (!Serial) delay(10);  // wait for serial port to open!
  
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }*/

  
}};