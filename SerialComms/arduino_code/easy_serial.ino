#define BAUD_RATE 9600   // (1) Optimized baud rate for stability & speed
#define BUFFER_SIZE 64     // Arduino default serial buffer size
#define MESSAGE " HELLO\n HELLO\n"  // Response message (5 chars + newline)

// Circular buffer to store incoming data
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) { }  // Wait for serial to initialize (for boards like Leonardo)
}

void loop() {
    readSerialData();
    Serial.print(MESSAGE);
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

        // (3) Simple flow control: process only if buffer has "S\n"
        if (strcmp(inputBuffer, "S\n") == 0) {
            sendData();
            bufferIndex = 0;  // Clear buffer after sending response
        }
    }
}

void sendData() {
    // (3) Software flow control: ensure space before writing
    while (Serial.availableForWrite() < sizeof(MESSAGE) - 1) {
        delay(1);  // Wait if TX buffer is full
    }
    
    Serial.print(MESSAGE);  // Send 5-character message with newline
}
