#ifndef SENSOR_h
#define SENSOR_h
#include <Arduino.h>
#include "MPU6050.h"
#include <BLEMidi.h>

class SENSOR {
private:
  uint8_t filteredExponentialValue;
  char _controllerNumber;
  char _channel;
  char _statusCode;
  unsigned long dataBuffer;
  std::string _sensorType;
public:
  SENSOR(const std::string sensorType, const uint8_t controllerNumber, const uint8_t pin = 0, const uint8_t intPin = 0);
  uint8_t _pin;
  uint8_t _intPin;
  uint8_t previousValue;
  uint8_t currentValue;
  int16_t filteredValue;
  uint8_t measuresCounter;
  bool isActive();
  int16_t getRawValue(MPU6050 sensor);
  uint8_t getMappedMidiValue(int16_t actualValue, int floor, int ceil);
  int16_t runBlockingAverageFilter(int measureSize, MPU6050 sensor, int gap = 1);
  int16_t runNonBlockingAverageFilter(int measureSize);
  int16_t runExponentialFilter(int measureSize, MPU6050 sensor, float alpha = 0.2);
  std::vector< uint8_t > getValuesBetweenRanges(uint8_t gap = 1);
  void setCurrentValue(uint8_t value);
  void setPreviousValue(uint8_t value);
  void setMeasuresCounter(uint8_t value);
  void setDataBuffer(int16_t value);

  void setMidiChannel(uint8_t channel) {
    _channel = channel;
  }

  void sendMidiMessage(BLEMidiServerClass serverInstance, char messageType[], uint8_t value, const char mode[] = "BLE") {
    if (strcmp(mode, "BLE") == 0) {
      if (strcmp(messageType, "controlChange") == 0) {
        serverInstance.controlChange(_channel, _controllerNumber, char(value));
      }
    }
    if (strcmp(mode, "Serial") == 0) {
      Serial.write(_statusCode);
      Serial.write(_controllerNumber);
      Serial.write(char(value));
    }
  }
};

#endif