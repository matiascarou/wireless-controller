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

  if (strcmp(_sensorType, "az") == 0) {
    return this->sensor.getAccelerationZ();
  }

  if (strcmp(_sensorType, "gx") == 0) {
    return this->sensor.getRotationX();
  }

  if (strcmp(_sensorType, "gy") == 0) {
    return this->sensor.getRotationY();
  }

  if (strcmp(_sensorType, "gz") == 0) {
    return this->sensor.getRotationZ();
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

std::vector< int > SENSOR::getValuesBetweenRanges(int gap) {
  // std::cout << "Current value: " << currentValue << "\t";
  // std::cout << "Previous value: " << previousValue << "\n";
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

/////////////////////////////////////////////////////////////////////////////
// std::vector< int > getRanges(int currentValue, int previousValue) {
//   int samples = currentValue - previousValue;
//   if (currentValue < previousValue) {
//       samples = previousValue - currentValue;
//   }
//   int index = previousValue;
//   std::vector< int > steps(samples + 1);
//   std::cout << "Array size: " << steps.size() << "\n";
//   std::generate(steps.begin(), steps.end(), [&index, currentValue, previousValue]() {
//     return currentValue > previousValue ? index++ : index--;
//   });
//   if (currentValue == previousValue) {
//       return steps;
//   }
//   steps.erase(steps.begin());
//   return steps;
// }
