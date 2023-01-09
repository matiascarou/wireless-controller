#include "Sensor.h"
#include "MPU6050.h"

SENSOR::SENSOR(const char* sensorType, const int controllerNumber, uint8_t pin, uint8_t int_pin) {
  _pin = pin;
  _sensorType = sensorType;
  _controllerNumber = controllerNumber;
  _intPin = int_pin;
  previousValue;
  currentValue;
}

void SENSOR::setCurrentValue(int value) {
  this->currentValue = value;
}

void SENSOR::setPreviousValue(int value) {
  this->previousValue = value;
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

int16_t SENSOR::getAverageValue(int measureSize, int gap, MPU6050 sensor) {
  int buffer = 0;
  for (int i = 0; i < measureSize; i++) {
    int16_t value = this->getRawValue(sensor);
    if (value < 0) {
      value = 0;
    }
    buffer += value;
    delay(gap);
  }
  const int16_t result = buffer / measureSize;
  return result;
}

int SENSOR::getMappedMidiValue(int16_t actualValue, int floor, int ceil) {
  return constrain(map(actualValue, floor, ceil, 0, 127), 0, 127);
}
