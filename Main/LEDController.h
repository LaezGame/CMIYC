class LEDController{
  public:
    byte pin;
    bool isOn;
  
  LEDController(){
    this->pin = 7;
  }

  LEDController(byte ipin){
    this->pin = ipin;
  }

  // SETTER
  void setPin(int newPin){
    this->pin = newPin;
  }

  void setState(bool shouldBeOn){
    this->isOn = shouldBeOn;
    digitalWrite(this->pin, shouldBeOn);
  }

  // GETTER
  int getPin(){
    return this->pin;
  }

  bool getState(){
    return this->isOn;
  }

  void ON(){
    this->setState(true);
  }

  void OUT(){
    this->setState(false);
  }

  void changeState(){
    this->setState(!this->getState());
  }
};