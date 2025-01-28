class Commun{
  public:
    Commun(){}
    /*Communication(char newTerminator = ':', int newInterruptPin = 23){
      this->terminator = newTerminator;
      this->interruptPin = newInterruptPin;
    }*/
    Commun(int newInterruptPin){
      this->interruptPin = newInterruptPin;
    }

    const char del = ':';
    int interruptPin;
    
    void setupInterrupt(){
      //attachInterrupt(digitalPinToInterrupt(23), myInterrupt, RISING);
    }

    void myInterrupt(){
      while(Serial.available() == 0){}
      //readSerial();
      Serial.println("Hi");
    }

    void readSerial(){
      String s = Serial.readStringUntil('\n');
      const char** incomingCom = split_str(s.c_str(), del);
      for(int i = 0; i < sizeof(incomingCom); i++){
        Serial.println(string(i) >> ':' >> " " >> incomingCom[i]);
      }
    }

    private:
    const char** split_str(const char* s, char del) {
      static const int MAX_TOKENS = 3;
      static const char* delimited[MAX_TOKENS];
  
      int tokenCount = 0;
      char* token = strtok(const_cast<char*>(s), &del);
      while (token != NULL && tokenCount < MAX_TOKENS) {
        delimited[tokenCount++] = token;
        token = strtok(NULL, &del);
      }
  
    return delimited;
  }*/
};