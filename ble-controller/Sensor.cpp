#include "Sensor.h"
#include "MPU6050.h"
#include <vector>
#include <algorithm>
#include <numeric>

SENSOR::SENSOR(std::string sensorType, const int controllerNumber, uint8_t pin, uint8_t intPin) {
  _pin = pin;
  _sensorType = sensorType;
  _controllerNumber = controllerNumber;
  _intPin = intPin;
  previousValue = 0;
  currentValue = 0;
  dataBuffer = 0;
  measuresCounter = 0;
  filteredExponentialValue = 0;
}

void SENSOR::setCurrentValue(int value) {
  this->currentValue = value;
}

void SENSOR::setPreviousValue(int value) {
  this->previousValue = value;
}

int16_t SENSOR::getRawValue(MPU6050 sensor) {

  if (_sensorType == "analogInput") {
    return analogRead(_pin);
  }

  if (_sensorType == "sonar") {
    const int16_t pulse = pulseIn(_pin, HIGH);
    const int16_t pulgadas = pulse / 147;
    return pulgadas;
  }

  if (_sensorType == "ax") {
    return sensor.getAccelerationX();
  }

  if (_sensorType == "ay") {
    return sensor.getAccelerationY();
  }

  if (_sensorType == "az") {
    return sensor.getAccelerationZ();
  }

  if (_sensorType == "gx") {
    return sensor.getRotationX();
  }

  if (_sensorType == "gy") {
    return sensor.getRotationY();
  }

  if (_sensorType == "gz") {
    return sensor.getRotationZ();
  }

  if (_sensorType == "analogInput") {
    return analogRead(_pin);
  }

  // if (strcmp(_sensorType, "analogInput") == 0) {
  //   return analogRead(_pin);
  // }

  return 0;
}

int16_t SENSOR::runBlockingAverageFilter(int measureSize, MPU6050 sensor, int gap) {
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

int16_t SENSOR::runNonBlockingAverageFilter(int measureSize) {
  return this->dataBuffer / measureSize;
}

int16_t SENSOR::runExponentialFilter(int measureSize, MPU6050 sensor, float alpha) {
  const int16_t rawValue = this->getRawValue(sensor);
  this->filteredExponentialValue = (alpha * rawValue) + (1 - alpha) * this->filteredExponentialValue;
  return this->filteredExponentialValue;
}

// int SENSOR::runKalmanFilter(Kalman kalmanFilterInstance) {
//   const int16_t rawValue = this->getRawValue(sensor);
//   const float filteredFloatValue = kalman.filter(sensorValue);
//   const int filteredIntValue = int(filteredFloatValue);
//   return filteredIntValue
// }

std::vector< int > SENSOR::getValuesBetweenRanges(int gap) {
  int samples = currentValue - previousValue;
  if (currentValue < previousValue) {
    samples = previousValue - currentValue;
  }
  int startValue = previousValue;
  std::vector< int > steps(samples / gap);
  if (gap > 1) {
    int increment = currentValue > previousValue ? gap : -gap;
    std::generate(steps.begin(), steps.end(), [startValue, increment]() mutable {
      auto value = startValue;
      startValue += increment;
      // TODO: Make this better
      if (value >= 124) {
        return 127;
      }
      if (value <= 2) {
        return 0;
      }
      return value;
    });
    return steps;
  } else {
    std::generate(steps.begin(), steps.end(), [&startValue, this, &gap]() {
      return this->currentValue > this->previousValue ? startValue += gap : startValue -= gap;
    });
  }
  if (currentValue == previousValue) {
    return steps;
  }
  return steps;
}

int SENSOR::getMappedMidiValue(int16_t actualValue, int floor, int ceil) {
  return constrain(map(actualValue, floor, ceil, 0, 127), 0, 127);
}