#include "Sensor.h"
#include "MPU6050.h"
#include "Adafruit_VL53L0X.h"
#include <math.h>
#include <vector>
#include <map>
#include <string>

static const int IMU_FLOOR = 700;
static const int IMU_CEIL = 15700;
static const int IMU_BASE_FILTER_THRESHOLD = 35;

static std::map<uint8_t, uint8_t> imuFilterResolution = {
  { 1, IMU_BASE_FILTER_THRESHOLD },
  { 2, 30 },
  { 3, 25 },
  { 4, 20 },
  { 5, 15 },
  { 6, 5 },
};

int16_t Sensor::getFilterThreshold(std::string &type) {
  static std::map<std::string, int> filterThresholdValues = {
    { "potentiometer", 20 },
    { "force", 1 },
    { "sonar", 1 },
    { "ax", IMU_BASE_FILTER_THRESHOLD },
    { "ay", IMU_BASE_FILTER_THRESHOLD },
    { "az", IMU_BASE_FILTER_THRESHOLD },
    { "gx", IMU_BASE_FILTER_THRESHOLD },
    { "gy", IMU_BASE_FILTER_THRESHOLD },
    { "gz", IMU_BASE_FILTER_THRESHOLD },
    { "infrared", 2 },
  };
  return filterThresholdValues[type];
}

static const int16_t SONAR_FLOOR = 40;

int16_t Sensor::getFloor(std::string &type) {
  static std::map<std::string, int> floorValues = {
    { "potentiometer", 20 },
    { "force", 500 },
    { "sonar", SONAR_FLOOR },
    { "ax", IMU_FLOOR },
    { "ay", IMU_FLOOR },
    { "az", IMU_FLOOR },
    { "gx", IMU_FLOOR },
    { "gy", IMU_FLOOR },
    { "gz", IMU_FLOOR },
    { "infrared", 70 },
  };
  return floorValues[type];
}

int16_t Sensor::getCeil(std::string &type) {
  static std::map<std::string, int> ceilValues = {
    { "potentiometer", 1000 },
    { "force", 1000 },
    { "sonar", SONAR_FLOOR + 30 },
    { "ax", IMU_CEIL },
    { "ay", IMU_CEIL },
    { "az", IMU_CEIL },
    { "gx", IMU_CEIL },
    { "gy", IMU_CEIL },
    { "gz", IMU_CEIL },
    { "infrared", 400 },
  };
  return ceilValues[type];
}

uint16_t Sensor::getDebounceThreshold(std::string &type) {
  static std::map<std::string, int> debounceThresholdValues = {
    { "force", 30 },
  };
  return debounceThresholdValues[type];
}

Sensor::Sensor(const std::string &sensorType, const uint8_t &controllerNumber, const uint8_t &pin, const uint8_t &intPin) {
  _pin = pin;
  _controllerNumber = controllerNumber;
  _channel = 0;
  _statusCode = 176;
  _intPin = intPin;
  _midiMessage = sensorType != "force" ? "controlChange" : "gate";
  _sensorType = sensorType;
  previousValue = 0;
  previousRawValue = 0;
  currentValue = 0;
  dataBuffer = 0;
  measuresCounter = 0;
  filteredExponentialValue = 0;
  _currentDebounceValue = 0;
  _previousDebounceValue = 0;
  isActive = false;
  toggleStatus = false;
  previousToggleStatus = toggleStatus;
  isAlreadyPressed = false;
  _floor = Sensor::getFloor(_sensorType);
  _ceil = Sensor::getCeil(_sensorType);
  _threshold = Sensor::getFilterThreshold(_sensorType);
  _debounceThreshold = Sensor::getDebounceThreshold(_sensorType);
  currentDebounceTimer = 0;
  previousDebounceTimer = 0;
  currentSwitchState = false;
  previousSwitchState = false;
  isDebounced = false;
  counter = 0;
  msb = 0;
  lsb = 0;
}

bool Sensor::isAboveThreshold() {
  return this->measuresCounter % this->_threshold == 0;
};

void Sensor::setCurrentValue(uint8_t value) {
  this->currentValue = value;
}

void Sensor::setThreshold(uint8_t value) {
  this->_threshold = value;
}

void Sensor::setThresholdBasedOnActiveSiblings(const uint8_t &amountOfActiveSiblings) {
  if (this->_sensorType == "ax" || this->_sensorType == "ay") {
    this->_threshold = imuFilterResolution[amountOfActiveSiblings];
  }
}

void Sensor::setMidiMessage(std::string value) {
  this->_midiMessage = value;
}

void Sensor::setPreviousValue(uint8_t value) {
  this->previousValue = value;
}

void Sensor::setPreviousRawValue(int16_t value) {
  this->previousRawValue = value;
}

void Sensor::setMidiChannel(uint8_t channel) {
  this->_channel = channel;
}

void Sensor::setCurrentDebounceValue(unsigned long timeValue) {
  this->_currentDebounceValue = timeValue;
}

void Sensor::setMeasuresCounter(uint8_t value) {
  this->measuresCounter = !value ? value : this->measuresCounter + value;
}

void Sensor::setDataBuffer(int16_t value) {
  this->dataBuffer = !value ? value : this->dataBuffer + value;
}

std::string Sensor::getSensorType() {
  return this->_sensorType;
}

bool Sensor::isSwitchActive() {
  const bool isSwitchActive = !!this->_intPin ? !!digitalRead(this->_intPin) : true;
  this->currentSwitchState = isSwitchActive;
  if (this->currentSwitchState != this->previousSwitchState) {
    this->isDebounced = false;
  }
  return isSwitchActive;
}

bool Sensor::isSwitchDebounced() {
  this->currentDebounceTimer = millis();
  if (_sensorType == "ax" || _sensorType == "ay") {
    if (!this->isDebounced) {
      if (this->currentDebounceTimer - this->previousDebounceTimer >= 500) {
        this->previousSwitchState = this->currentSwitchState;
        this->isDebounced = true;
        this->previousDebounceTimer = this->currentDebounceTimer;
        return true;
      }
      return false;
    }
    this->previousDebounceTimer = this->currentDebounceTimer;
    return true;
  }
  return true;
}

int16_t Sensor::getRawValue(MPU6050 *accelgyro, Adafruit_VL53L0X *lox) {

  if (_sensorType == "potentiometer" || _sensorType == "force") {
    return analogRead(_pin);
  }

  if (_sensorType == "sonar") {
    const uint32_t pulse = pulseIn(_pin, HIGH);
    const int16_t inches = pulse / 147;
    return inches;
  }

  if (_sensorType == "infrared") {
    VL53L0X_RangingMeasurementData_t measure;

    lox->rangingTest(&measure, false);

    return measure.RangeStatus != 4 && measure.RangeMilliMeter >= _floor / 2 ? measure.RangeMilliMeter : this->previousRawValue;
  }

  if (_sensorType == "ax") {
    const int16_t rawValue = accelgyro->getAccelerationX();
    return constrain(rawValue, 0, _ceil);
  }

  if (_sensorType == "ay") {
    const int16_t rawValue = accelgyro->getAccelerationY();
    return constrain(rawValue, 0, _ceil);
  }

  if (_sensorType == "az") {
    const int16_t rawValue = accelgyro->getAccelerationZ();
    return constrain(rawValue, 0, _ceil);
  }

  if (_sensorType == "gx") {
    const int16_t rawValue = accelgyro->getRotationX();
    return constrain(rawValue, 0, _ceil);
  }

  if (_sensorType == "gy") {
    const int16_t rawValue = accelgyro->getRotationY();
    return constrain(rawValue, 0, _ceil);
  }

  if (_sensorType == "gz") {
    const int16_t rawValue = accelgyro->getRotationZ();
    return constrain(rawValue, 0, _ceil);
  }

  return 0;
}


int16_t Sensor::runNonBlockingAverageFilter() {
  return this->dataBuffer / this->_threshold;
}

int16_t Sensor::runBlockingAverageFilter(int measureSize, MPU6050 *accelgyro, Adafruit_VL53L0X *lox, int gap) {
  int buffer = 0;
  for (int i = 0; i < measureSize; i++) {
    int16_t value = this->getRawValue(accelgyro, lox);
    if (value < 0) {
      value = 0;
    }
    buffer += value;
    delayMicroseconds(gap);
  }
  const int16_t result = buffer / measureSize;
  return result;
}

int16_t Sensor::runExponentialFilter(MPU6050 *accelgyro, Adafruit_VL53L0X *lox) {
  static const float alpha = 0.5;
  const int16_t rawValue = this->getRawValue(accelgyro, lox);
  const float filteredValue = rawValue * alpha + (1 - alpha) * rawValue;
  return int(filteredValue);
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

int Sensor::getMappedMidiValue(int16_t actualValue, int floor, int ceil) {
  if (floor && ceil) {
    return constrain(map(actualValue, floor, ceil, 0, 127), 0, 127);
  }
  if (this->_midiMessage == "pitchBend") {
    const int pitchBendValue = constrain(map(actualValue, _floor, _ceil, 8191, 16383), 8191, 16383);
    int shiftedValue = pitchBendValue << 1;
    this->msb = highByte(shiftedValue);
    this->lsb = lowByte(shiftedValue) >> 1;
    return pitchBendValue;
  }
  return constrain(map(actualValue, _floor, _ceil, 0, 127), 0, 127);
}

void Sensor::debounce(MPU6050 *accelgyro, Adafruit_VL53L0X *lox) {
  if (_sensorType == "force") {
    this->previousToggleStatus = this->toggleStatus;
    if (this->_currentDebounceValue - this->_previousDebounceValue >= _debounceThreshold) {
      const int16_t rawValue = this->getRawValue(accelgyro, lox);
      const uint8_t sensorMappedValue = this->getMappedMidiValue(rawValue);
      this->toggleStatus = !!sensorMappedValue ? true : false;
      this->_previousDebounceValue = this->_currentDebounceValue;
    }
  }
}

bool Sensor::isSibling(const std::vector<std::string> &SIBLINGS) {
  const std::vector<std::string>::const_iterator it = std::find(SIBLINGS.begin(), SIBLINGS.end(), this->_sensorType);
  return it != SIBLINGS.end();
}

void Sensor::sendSerialMidiMessage(HardwareSerial *Serial2) {
  if (this->_midiMessage == "controlChange" && this->currentValue != this->previousValue) {
    Sensor::writeSerialMidiMessage(this->_statusCode, this->_controllerNumber, this->currentValue, Serial2);
  }
  if (this->_midiMessage == "gate" && this->toggleStatus != this->previousToggleStatus) {
    if (this->toggleStatus) {
      Sensor::writeSerialMidiMessage(144, 60, 127, Serial2);
    } else {
      Sensor::writeSerialMidiMessage(128, 60, 127, Serial2);
    }
  }
}

void Sensor::run(MPU6050 *accelgyro, Adafruit_VL53L0X *lox, const uint8_t &activeSiblings) {
  int16_t rawValue = this->getRawValue(accelgyro, lox);
  this->setPreviousRawValue(rawValue);
  this->setDataBuffer(rawValue);
  this->setMeasuresCounter(1);
  if (this->isAboveThreshold()) {
    const unsigned long currentDebounceValue = millis();
    this->setCurrentDebounceValue(currentDebounceValue);
    const int16_t averageValue = this->runNonBlockingAverageFilter();
    const uint8_t sensorMappedValue = this->getMappedMidiValue(averageValue);
    this->setPreviousValue(this->currentValue);
    this->setCurrentValue(sensorMappedValue);
    this->debounce(accelgyro, lox);
    this->sendSerialMidiMessage(&Serial2);
    this->setMeasuresCounter(0);
    this->setDataBuffer(0);
    this->setThresholdBasedOnActiveSiblings(activeSiblings);
  }
}

/**
  * TODO: Test for ESP32 device.
  **/
// void Sensor::sendBleMidiMessage(BLEMidiServerClass *serverInstance) {
//   if (this->_midiMessage == "controlChange") {
//     if (this->currentValue != this->previousValue) {
//       serverInstance->controlChange(_channel, _controllerNumber, char(this->currentValue));
//     }
//   }
// if (this->_midiMessage == "gate") {
//   if (this->toggleStatus != this->previousToggleStatus) {
//     if (this->toggleStatus) {
//       serverInstance->noteOn(_channel, char(60), char(127));
//     } else {
//       serverInstance->noteOff(_channel, char(60), char(127));
//     }
//   }
// }
//   if (this->_midiMessage == "pitchBend") {
//     if (this->currentValue != this->previousValue) {
//       serverInstance->pitchBend(_channel, this->lsb, this->msb);
//     }
//   }
// }