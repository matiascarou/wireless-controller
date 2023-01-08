// LED.cpp
#include "Sensor.h"
#include "MPU6050.h"

SENSOR::SENSOR(const char* sensorType, const int controllerNumber, uint8_t pin) {
  _pin = pin;
  previousValue = 0;
  actualValue = 0;
  _sensorType = sensorType;
  _controllerNumber = controllerNumber;
  // pinMode(ledPin, OUTPUT);
}

uint16_t SENSOR::getRawValue(MPU6050 sensor) {


  if (strcmp(_sensorType, "analogInput") == 0) {
    return analogRead(_pin);
  }

  if (strcmp(_sensorType, "sonar") == 0) {
    const int pulse = pulseIn(_pin, HIGH);
    const int pulgadas = pulse / 147;
    return pulgadas;
  }

  if (strcmp(_sensorType, "ax") == 0) {
    return sensor.getAccelerationX();
  }

  if (strcmp(_sensorType, "ay") == 0) {
    return sensor.getAccelerationY();
  }

  if (strcmp(_sensorType, "gx") == 0) {
    return sensor.getRotationX();
  }

  if (strcmp(_sensorType, "gy") == 0) {
    return sensor.getRotationY();
  }

  return 0;
}

int SENSOR::getMappedMidiValue(int16_t actualValue, int floor, int ceil) {
  return constrain(map(actualValue, floor, ceil, 0, 127), 0, 127);
}

int SENSOR::getAverageValue(unsigned long value, int measureSize, int gap) {
  int buffer = 0;
  for (int i = 0; i < measureSize; i++) {
    value = constrain(map(value, 0, 16500, 0, 127), 0, 127);
    buffer += value;
    delay(gap);
  }
  return buffer / measureSize;
}