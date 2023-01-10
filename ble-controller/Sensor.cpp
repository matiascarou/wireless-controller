#include "Sensor.h"
#include "MPU6050.h"
#include <vector>
#include <algorithm>
#include <numeric>

SENSOR::SENSOR(const char* sensorType, const int controllerNumber, uint8_t pin, uint8_t int_pin) {
  _pin = pin;
  _sensorType = sensorType;
  _controllerNumber = controllerNumber;
  _intPin = int_pin;
  previousValue = 0;
  currentValue = 0;
  dataBuffer = 0;
  measuresCounter = 0;
}

void SENSOR::setCurrentValue(int value) {
  this->currentValue = value;
}

void SENSOR::setPreviousValue(int value) {
  this->previousValue = value;
}

int16_t SENSOR::getRawValue(MPU6050 sensor) {

  if (strcmp(_sensorType, "analogInput") == 0) {
    return analogRead(_pin);
  }

  if (strcmp(_sensorType, "sonar") == 0) {
    const int pulse = pulseIn(_pin, HIGH);
    const int pulgadas = pulse / 147;
    return pulgadas;
  }

  if (strcmp(_sensorType, "ax") == 0) {
    return this->sensor.getAccelerationX();
  }

  if (strcmp(_sensorType, "ay") == 0) {
    return this->sensor.getAccelerationY();
  }

  if (strcmp(_sensorType, "gx") == 0) {
    return this->sensor.getRotationX();
  }

  if (strcmp(_sensorType, "gy") == 0) {
    return this->sensor.getRotationY();
  }

  return 0;
}

int16_t SENSOR::getAverageValue(int measureSize, MPU6050 sensor, char* behaviour, int gap) {
  if (behaviour == "non-blocking") {
    return this->dataBuffer / measureSize;
  } else {
    int buffer = 0;
    for (int i = 0; i < measureSize; i++) {
      int16_t value = this->getRawValue(sensor);
      if (value < 0) {
        value = 0;
      }
      buffer += value;
      delayMicroseconds(gap);
    }
    const int16_t result = buffer / measureSize;
    return result;
  }
}

int SENSOR::getValuesBetweenRanges() {
  if (this->currentValue > this->previousValue) {
    int samples = 0;
    Serial.print("Current value is: ");
    Serial.print(this->currentValue);
    Serial.print("\t Previous value is: ");
    Serial.print(this->previousValue);
    samples = this->currentValue - this->previousValue;
    std::vector<int> steps(samples);
    std::generate(steps.begin(), steps.end(), []() {
      static int i = 0;
      return i += 1;
    });
    Serial.print("\tSTART STEP: ");
    const int startStep = *steps.begin();
    Serial.print(startStep);
    Serial.print("\tEND STEP: ");
    const int endStep = steps.back();
    Serial.println(endStep);

    for (int item : steps) {
      Serial.print("ITEM: ");
      Serial.print(item);
      Serial.print("\t");
    }
    Serial.print("\n");
    this->previousValue = this->currentValue;
  }

  return 0;
}

int SENSOR::getMappedMidiValue(int16_t actualValue, int floor, int ceil) {
  return constrain(map(actualValue, floor, ceil, 0, 127), 0, 127);
}
