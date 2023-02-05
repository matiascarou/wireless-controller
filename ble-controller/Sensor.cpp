#include "Sensor.h"
#include "MPU6050.h"
#include <BLEMidi.h>

// struct Value {
//   int16_t floor;
//   int16_t ceil;
//   int16_t threshold;
// };

// static std::map<std::string, Value> values = {
//   { "potentiometer", { 20, 1023, 30 } },
//   { "force", { 20, 1023, 20 } },
//   { "sonar", { 6, 30, 40 } },
//   { "ax", { 100, 15500, 50 } },
//   { "ay", { 100, 15500, 50 } },
//   { "gx", { 100, 15500, 50 } },
//   { "gy", { 100, 15500, 50 } },
// };

// int16_t getValueFromMapObject(std::map<std::string, Value> mapObject, std::string &type) {

// }

int16_t Sensor::getFilterThreshold(std::string &type) {
  static std::map<std::string, int> filterThresholdValues = {
    { "potentiometer", 40 },
    { "force", 5 },
    { "sonar", 40 },
    { "ax", 50 },
    { "ay", 50 },
    { "gx", 50 },
    { "gy", 50 },
  };
  return filterThresholdValues[type];
}

int16_t Sensor::getFloor(std::string &type) {
  static std::map<std::string, int> floorValues = {
    { "potentiometer", 20 },
    { "force", 60 },
    { "sonar", 6 },
    { "ax", 200 },
    { "ay", 200 },
    { "gx", 200 },
    { "gy", 200 },
  };
  return floorValues[type];
}

int16_t Sensor::getCeil(std::string &type) {
  static std::map<std::string, int> ceilValues = {
    { "potentiometer", 1023 },
    { "force", 1023 },
    { "sonar", 30 },
    { "ax", 15500 },
    { "ay", 15500 },
    { "gx", 15500 },
    { "gy", 15500 },
  };
  return ceilValues[type];
}

Sensor::Sensor(const std::string &sensorType, const uint8_t &controllerNumber, const uint8_t &pin, const uint8_t &intPin) {
  _pin = pin;
  _controllerNumber = char(controllerNumber);
  _channel = char(0);
  _statusCode = char(176);
  _intPin = intPin;
  _midiMessage = sensorType != "force" ? "controlChange" : "gate";
  _sensorType = sensorType;
  previousValue = 0;
  currentValue = 0;
  dataBuffer = 0;
  measuresCounter = 0;
  filteredExponentialValue = 0;
  _floor = Sensor::getFloor(_sensorType);
  _ceil = Sensor::getCeil(_sensorType);
  _threshold = Sensor::getFilterThreshold(_sensorType);
  // _floor = getValueFromMapObject(floorValues, _sensorType);
  // _ceil = getValueFromMapObject(ceilValues, _sensorType);
  // _threshold = getValueFromMapObject(thresholdValues, _sensorType);
  isActive = false;
}

bool Sensor::isSwitchActive() {
  return this->_intPin ? !!digitalRead(this->_intPin) : true;
};

bool Sensor::isAboveThreshold() {
  return this->measuresCounter % _threshold == 0;
};

void Sensor::setCurrentValue(uint8_t value) {
  this->currentValue = value;
}

void Sensor::setThreshold(uint8_t value) {
  this->_threshold = value;
}

void Sensor::setMidiMessage(std::string value) {
  this->_midiMessage = value;
}

void Sensor::setPreviousValue(uint8_t value) {
  this->previousValue = value;
}

void Sensor::setMidiChannel(uint8_t channel) {
  this->_channel = channel;
}

void Sensor::setMeasuresCounter(uint8_t value) {
  if (value == 0) {
    this->measuresCounter = value;
  } else {
    this->measuresCounter += value;
  }
}

void Sensor::setDataBuffer(int16_t value) {
  if (value == 0) {
    this->dataBuffer = value;
  } else {
    this->dataBuffer += value;
  }
}

int16_t Sensor::getRawValue(MPU6050 &accelgyro) {

  if (_sensorType == "potentiometer" || _sensorType == "force") {
    return analogRead(_pin);
  }

  if (_sensorType == "sonar") {
    const int16_t pulse = pulseIn(_pin, HIGH);
    const int16_t pulgadas = pulse / 147;
    return pulgadas;
  }

  if (_sensorType == "ax") {
    const int16_t rawValue = accelgyro.getAccelerationX();
    return constrain(rawValue, 0, 15500);
  }

  if (_sensorType == "ay") {
    const int16_t rawValue = accelgyro.getAccelerationY();
    return constrain(rawValue, 0, 15500);
  }

  if (_sensorType == "az") {
    const int16_t rawValue = accelgyro.getAccelerationZ();
    return constrain(rawValue, 0, 15500);
  }

  if (_sensorType == "gx") {
    const int16_t rawValue = accelgyro.getRotationX();
    return constrain(rawValue, 0, 15500);
  }

  if (_sensorType == "gy") {
    const int16_t rawValue = accelgyro.getRotationY();
    return constrain(rawValue, 0, 15500);
  }

  if (_sensorType == "gz") {
    const int16_t rawValue = accelgyro.getRotationZ();
    return constrain(rawValue, 0, 15500);
  }

  return 0;
}

int16_t Sensor::runBlockingAverageFilter(int measureSize, MPU6050 &accelgyro, int gap) {
  int buffer = 0;
  for (int i = 0; i < measureSize; i++) {
    int16_t value = this->getRawValue(accelgyro);
    if (value < 0) {
      value = 0;
    }
    buffer += value;
    delayMicroseconds(gap);
  }
  const int16_t result = buffer / measureSize;
  return result;
}

int16_t Sensor::runNonBlockingAverageFilter() {
  return this->dataBuffer / _threshold;
}

int16_t Sensor::runExponentialFilter(int measureSize, MPU6050 &accelgyro, float alpha) {
  const int16_t rawValue = this->getRawValue(accelgyro);
  this->filteredExponentialValue = (alpha * rawValue) + (1 - alpha) * this->filteredExponentialValue;
  return this->filteredExponentialValue;
}

std::vector< uint8_t > Sensor::getValuesBetweenRanges(uint8_t gap) {
  uint8_t samples = 1;
  if (currentValue > previousValue) {
    samples = currentValue - previousValue;
  }
  if (currentValue < previousValue) {
    samples = previousValue - currentValue;
  }
  std::vector< uint8_t > steps(samples / gap);
  uint8_t startValue = previousValue;
  std::generate(steps.begin(), steps.end(), [&startValue, this, &gap]() {
    if (this->currentValue > this->previousValue) {
      return startValue += gap;
    }
    if (this->currentValue < this->previousValue) {
      return startValue -= gap;
    }
    return startValue;
  });
  return steps;
}

uint8_t Sensor::getMappedMidiValue(int16_t actualValue, int floor, int ceil) {
  if (floor && ceil) {
    return constrain(map(actualValue, floor, ceil, 0, 127), 0, 127);
  }
  return constrain(map(actualValue, _floor, _ceil, 0, 127), 0, 127);
}

void Sensor::sendMidiMessage(BLEMidiServerClass &serverInstance, uint8_t value, const char mode[]) {
  if (strcmp(mode, "BLE") == 0) {
    if (_midiMessage == "controlChange") {
      serverInstance.controlChange(_channel, _controllerNumber, char(value));
    }
    if (_midiMessage == "gate") {
      if (!!value && !this->isActive) {
        serverInstance.noteOn(_channel, char(60), char(127));
        this->isActive = true;
      }
      if (!value && this->isActive) {
        serverInstance.noteOff(_channel, char(60), char(127));
        this->isActive = false;
      }
    }
  }
  if (strcmp(mode, "Serial") == 0) {
    Serial.write(_statusCode);
    Serial.write(_controllerNumber);
    Serial.write(char(value));
  }1    
}

// int Sensor::runKalmanFilter(Kalman kalmanFilterInstance) {
//   const int16_t rawValue = this->getRawValue(accelgyro);
//   const float filteredFloatValue = kalman.filter(sensorValue);
//   const int filteredIntValue = int(filteredFloatValue);
//   return filteredIntValue
// }

// std::vector< uint8_t > Sensor::getValuesBetweenRanges(uint8_t gap) {
//   uint8_t samples = 1;
//   if (currentValue > previousValue) {
//     samples = currentValue - previousValue;
//   }
//   if (currentValue < previousValue) {
//     samples = previousValue - currentValue;
//   }
//   std::vector< uint8_t > steps(samples / gap);
//   uint8_t startValue = previousValue;
//   uint8_t increment = currentValue > previousValue ? gap : -gap;
//   if (gap > 1) {
//     std::generate(steps.begin(), steps.end(), [startValue, increment]() mutable {
//       uint8_t value = startValue;
//       startValue += increment;
//       // TODO: Make this better
//       if (value >= 124) {
//         return (uint8_t)127;
//       }
//       if (value <= 2) {
//         return (uint8_t)0;
//       }
//       return value;
//     });
//     return steps;
//   } else {
//     std::generate(steps.begin(), steps.end(), [&startValue, this, &gap]() {
//       if (this->currentValue > this->previousValue) {
//         return startValue += gap;
//       }
//       if (this->currentValue < this->previousValue) {
//         return startValue -= gap;
//       }
//       return startValue;
//     });
//     return steps;
//   }
// }