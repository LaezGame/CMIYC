class Motor{
public:
  int En, IN1, IN2;
  bool reverseDirection;

  Motor(){}
  Motor(int iEn,int INA,int INB){
    this->En = iEn;
    this->IN1 = INA;
    this->IN2 = INB;
  }
  Motor(int iEn,int INA,int INB, bool reverse){
    this->En = iEn;
    this->IN1 = INA;
    this->IN2 = INB;
    this->reverseDirection = reverse;
  }

  // SETTER
  void setEN(int iEn){
    En = iEn;
  }

  void setIN1(int IN){
    IN1 = IN;
  }

  void setIN2(int IN){
    IN2 = IN;
  }

  // GETTER
  int getEN(){
    return this->En;
  }

  int getIN1(){
    return this->IN1;
  }

  int getIN2(){
    return this->IN2;
  }

  // METHODS

  void setDirection(int direction){
    if(direction > 0){
    digitalWrite(IN1, 255);
    digitalWrite(IN2, 0);}
    else if(direction < 0){
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 255);}
    else if(direction == 0){
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 0);}



  }

  void setSpeed(byte speed){
    digitalWrite(En, speed);
  }
};