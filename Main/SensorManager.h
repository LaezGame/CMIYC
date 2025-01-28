#include "InfraredSensor.h"
#include "GyroSensor.h"

class SensorManager{
  public:
    InfraredSensor infraredSensors[6];
    GyroSensor gyroSensor;

  SensorManager(){
    
  }

  SensorManager(byte numberOfIR){
    for(int i = 0; i < numberOfIR && i < 6; i++){
      this->infraredSensors[i] = InfraredSensor(i);
    }
    gyroSensor = GyroSensor();
  }

  int measureDistance(byte numberOfIR){
    return (int)this->infraredSensors[numberOfIR].measureDistance();
  }

  int getGyroValue(char axis){
    return 0;
  }

  int getSensorValue(String typeOfSensor, int numberOfSensor, char axis){
    if(typeOfSensor.equals("IR")){
      int16_t d = infraredSensors[numberOfSensor].measureDistance();
      return d;  
    }
    else if(typeOfSensor.equals("GYRO")){

    }
  }

  // SETTER
  void setSpecificIRSensor(byte numberOfIR, InfraredSensor newSensor){
    this->infraredSensors[numberOfIR] = newSensor;
  }

  // GETTER
  InfraredSensor getSpecificIRSensors(int numberOfIR){
    return this->infraredSensors[numberOfIR];
  } 

};
