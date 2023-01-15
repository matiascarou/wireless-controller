#ifndef SENSOR_h
#define SENSOR_h
#include <Arduino.h>
#include "MPU6050.h"
#include <vector>
#include <BLEMidi.h>

class SENSOR {
private:
  std::string _sensorType;
  int filteredExponentialValue;
public:
  SENSOR(std::string sensorType, const int controllerNumber, const uint8_t pin = 0, const uint8_t intPin = 0);
  int _controllerNumber;
  uint8_t _pin;
  uint8_t _intPin;
  int previousValue;
  int currentValue;
  unsigned long dataBuffer;
  int16_t filteredValue;
  int measuresCounter;
  int16_t getRawValue(MPU6050 sensor);
  int getMappedMidiValue(int16_t actualValue, int floor, int ceil);
  int16_t runBlockingAverageFilter(int measureSize, MPU6050 sensor, int gap = 1);
  int16_t runNonBlockingAverageFilter(int measureSize);
  int16_t runExponentialFilter(int measureSize, MPU6050 sensor, float alpha = 0.2);
  std::vector< int > getValuesBetweenRanges(int gap = 1);
  void setCurrentValue(int value);
  void setPreviousValue(int value);

  void sendMidiMessage(BLEMidiServerClass serverInstance, std::string messageType, int value) {
    if (messageType == "controlChange") {
      serverInstance.controlChange(0, _controllerNumber, value);
    }
  }
};

#endif