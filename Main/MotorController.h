#include "Motor.h"

class MotorManager{
  public:
  Motor motorL;
  Motor motorR;

  MotorManager(){
    motorL = Motor(29, 30, 31);
    motorR = Motor(36, 35, 34);

  }

  // SETTER
  void setMotorL(Motor motor){
    this->motorL = motor;
  }

  void setMotorR(Motor motor){
    this->motorR = motor;
  }

  // GETTER
  Motor getMotorL(){
    return this->motorL;
  }

  Motor getMotorR(){
    return this->motorR;
  }

  // METHODS
  void forward(int speed){
    this->motorL.setSpeed(speed);
    this->motorL.setSpeed(speed);
  }
};


