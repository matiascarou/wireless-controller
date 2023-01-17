#include "Sensor.h"
#include "MPU6050.h"

SENSOR::SENSOR(const std::string sensorType, const uint8_t controllerNumber, uint8_t pin, uint8_t intPin) {
  _pin = pin;
  _controllerNumber = char(controllerNumber);
  _channel = char(0);
  _statusCode = char(176);
  _intPin = intPin;
  previousValue = 0;
  currentValue = 0;
  dataBuffer = 0;
  measuresCounter = 0;
  filteredExponentialValue = 0;
  _sensorType = sensorType;
}

void SENSOR::setCurrentValue(uint8_t value) {
  this->currentValue = value;
}

void SENSOR::setPreviousValue(uint8_t value) {
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

std::vector< uint8_t > SENSOR::getValuesBetweenRanges(uint8_t gap) {
  uint8_t samples = currentValue - previousValue;
  if (currentValue < previousValue) {
    samples = previousValue - currentValue;
  }
  uint8_t startValue = previousValue;
  std::vector< uint8_t > steps(samples / gap);
  // if (gap > 1) {
  uint8_t increment = currentValue > previousValue ? gap : -gap;
  //   std::generate(steps.begin(), steps.end(), [startValue, increment]() mutable {
  //     uint8_t value = startValue;
  //     startValue += increment;
  //     // TODO: Make this better
  //     if (value >= 124) {
  //       return (uint8_t)127;
  //     }
  //     if (value <= 2) {
  //       return (uint8_t)0;
  //     }
  //     return value;
  //   });
  //   return steps;
  // } else {
  std::generate(steps.begin(), steps.end(), [&startValue, this, &gap]() {
    return this->currentValue > this->previousValue ? startValue += gap : startValue -= gap;
  });
  // }
  return steps;
}

uint8_t SENSOR::getMappedMidiValue(int16_t actualValue, int floor, int ceil) {
  return constrain(map(actualValue, floor, ceil, 0, 127), 0, 127);
}