#ifndef SENSOR_h
#define SENSOR_h
#include <Arduino.h>
#include "MPU6050.h"
#include <BLEMidi.h>
// #include <vector>

class SENSOR {
private:
  uint8_t filteredExponentialValue;
  uint8_t _controllerNumber;
  std::string _sensorType;
public:
  SENSOR(const std::string sensorType, const uint8_t controllerNumber, const uint8_t pin = 0, const uint8_t intPin = 0);
  uint8_t _pin;
  uint8_t _intPin;
  uint8_t previousValue;
  uint8_t currentValue;
  unsigned long dataBuffer;
  int16_t filteredValue;
  int measuresCounter;
  int16_t getRawValue(MPU6050 sensor);
  uint8_t getMappedMidiValue(int16_t actualValue, int floor, int ceil);
  int16_t runBlockingAverageFilter(int measureSize, MPU6050 sensor, int gap = 1);
  int16_t runNonBlockingAverageFilter(int measureSize);
  int16_t runExponentialFilter(int measureSize, MPU6050 sensor, float alpha = 0.2);
  std::vector< uint8_t > getValuesBetweenRanges(uint8_t gap = 1);
  void setCurrentValue(uint8_t value);
  void setPreviousValue(uint8_t value);

  void sendMidiMessage(BLEMidiServerClass serverInstance, std::string messageType, uint8_t value, std::string mode = "BLE") {
    if (mode == "BLE") {
      if (messageType == "controlChange") {
        serverInstance.controlChange(0, _controllerNumber, value);
      }
    } else {
      Serial.write(176);
      Serial.write(_controllerNumber);
      Serial.write(value);
    }
  }
};

#endif