#ifndef Sensor_h
#define Sensor_h
#include <Arduino.h>
#include "MPU6050.h"
#include <BLEMidi.h>

class Sensor {
private:
  uint8_t filteredExponentialValue;
  char _controllerNumber;
  char _channel;
  char _statusCode;
  int16_t _floor;
  int16_t _threshold;
  int16_t _ceil;
  unsigned long dataBuffer;
  uint8_t measuresCounter;
  std::string _sensorType;
  int16_t getFloor();
  int16_t getThreshold();
  int16_t getCeil();
  int16_t getValueFromMapObject(std::map<std::string, int16_t> &values);
public:
  Sensor(const std::string sensorType, const uint8_t controllerNumber, const uint8_t pin = 0, const uint8_t intPin = 0);
  uint8_t _pin;
  uint8_t _intPin;
  uint8_t previousValue;
  uint8_t currentValue;
  int16_t filteredValue;
  bool isActive();
  bool isAboveThreshold();
  int16_t getRawValue(MPU6050 sensor);
  uint8_t getMappedMidiValue(int16_t actualValue, int floor = 0, int ceil = 0);
  int16_t runBlockingAverageFilter(int measureSize, MPU6050 sensor, int gap = 1);
  int16_t runNonBlockingAverageFilter();
  int16_t runExponentialFilter(int measureSize, MPU6050 sensor, float alpha = 0.2);
  std::vector< uint8_t > getValuesBetweenRanges(uint8_t gap = 1);
  void setCurrentValue(uint8_t value);
  void setPreviousValue(uint8_t value);
  void setMeasuresCounter(uint8_t value);
  void setDataBuffer(int16_t value);
  void setThreshold(uint8_t value);

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

  static std::vector<Sensor> initializeSensors() {
    std::vector<Sensor> SENSORS = {
      Sensor("analogInput", 102, A0),
      Sensor("analogInput", 103, A3),
      Sensor("ax", 105, 0, 18),
      Sensor("ay", 106, 0, 19)
    };
    return SENSORS;
    /**
    * Comment out lines below for STM32 + Xbees support
    * BLE approach will not work with the STM microcontroller
    **/
    //   std::vector<Sensor> SENSORS = {
    //     Sensor("analogInput", 102, PA0),
    //     Sensor("analogInput", 103, PA1),
    //     Sensor("analogInput", 103, PA1),
    //     Sensor("ax", 105, 0, PB12),
    //     Sensor("ay", 106, 0, PB14),
    //     Sensor("gx", 105, 0, PB13),
    //     Sensor("gy", 106, 0, PB3),
    //     Sensor("sonar", 107, PB15, PB5)
    //   };
    //   return SENSORS;
  }
};

#endif