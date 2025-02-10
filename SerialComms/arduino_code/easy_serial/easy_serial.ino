#include "Motor.h"

#define BAUD_RATE 9600   // (1) Optimized baud rate for stability & speed
#define BUFFER_SIZE 64     // Arduino default serial buffer size

// Circular buffer to store incoming data
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;
const char del = ':';

Motor motorL;
Motor motorR;

void setup() {
    motorL = Motor(29, 30, 31);
    motorR = Motor(36, 35, 34);
  
    Serial.begin(BAUD_RATE);
    while (!Serial) { }  // Wait for serial to initialize (for boards like Leonardo)
}

void loop() {
    readSerialData();
    delay(100);
}

char** split_str(const char* s, char del) {
      static const int MAX_TOKENS = 2;
      static const char* delimited[MAX_TOKENS];
  
      int tokenCount = 0;
      char* token = strtok(const_cast<char*>(s), &del);
      while (token != NULL && tokenCount < MAX_TOKENS) {
        delimited[tokenCount++] = token;
        token = strtok(NULL, &del);
      }
  
    return delimited;
  }

void readSerialData() {
    while (Serial.available() > 0) {
        char receivedChar = Serial.read();
        
        // (2) Prevent buffer overflow
        if (bufferIndex < BUFFER_SIZE - 1) {
            inputBuffer[bufferIndex++] = receivedChar;
            inputBuffer[bufferIndex] = '\0';  // Null-terminate the string
        } else {
            bufferIndex = 0;  // Reset buffer if overflow risk
        }

        // (3) Process if newline
        if (receivedChar == '\n' && bufferIndex > 0) {
            char** incomingCom = split_str(inputBuffer, del);
            if (incomingCom[0][0] == 'R') {
                memmove(incomingCom[0], incomingCom[0]+1, strlen(incomingCom[0])); // remove first character 'R'
                if (atoi(incomingCom[0]) > 0) motorR.setDirection(atoi(incomingCom[2]));
                if (atoi(incomingCom[1]) > 0) motorR.setSpeed(atoi(incomingCom[3]));
            } else if (incomingCom[0][0] == 'L') {
                memmove(incomingCom[0], incomingCom[0]+1, strlen(incomingCom[0])); // remove first character 'L'
                if (atoi(incomingCom[0]) > 0) motorL.setDirection(atoi(incomingCom[0]));
                if (atoi(incomingCom[1]) > 0) motorL.setSpeed(atoi(incomingCom[1]));
            }
            //Serial.println(inputBuffer);
            for(int i = 0; i < sizeof(incomingCom); i++){
                Serial.println(incomingCom[i]);
            }
            bufferIndex = 0;  // Clear buffer after processing
        }
    }
}

void sendData(const char* MESSAGE) {
    // (3) Software flow control: ensure space before writing
    while (Serial.availableForWrite() < sizeof(MESSAGE) - 1) {
        delay(1);  // Wait if TX buffer is full
    }
    
    Serial.println(MESSAGE);  // Send 5-character message with newline
}
