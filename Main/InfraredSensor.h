class InfraredSensor {
public:
  int sensorPin;

  InfraredSensor() {
    this->sensorPin = 0;
  }

  InfraredSensor(int newPin) {
    this->sensorPin = newPin;
  }

  // SETTER
  void setSensorPin(int newPin) {
    sensorPin = newPin;
  }

  // GETTER
  int getSensorPin() {
    return this->sensorPin;
  }

  int measureDistance() {
    int t = pulseIn(this->sensorPin, HIGH);

    //int t = 3;
    if (t == 0) {
      // pulseIn() did not detect the start of a pulse within 1 second.
      Serial.println("timeout");
      return 0;
    } else if (t > 1850) {
      // No detection.
      Serial.println(-1);
      return -1;
    } else {
      // Valid pulse width reading. Convert pulse width in microseconds to distance in millimeters.
      int16_t d = (t - 1000) * 3 / 4;

      // Limit minimum distance to 0.
      if (d < 0) { d = 0; }

      Serial.print(d);
      Serial.println(" mm");
      return d;
    }
  }
};
