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
  bool isActive;
  std::string _midiMessage;
  std::string _sensorType;
  int16_t getFloor(std::string &type);
  int16_t getFilterThreshold(std::string &type);
  int16_t getCeil(std::string &type);
public:
  Sensor(const std::string sensorType, const uint8_t controllerNumber, const uint8_t pin = 0, const uint8_t intPin = 0);
  uint8_t _pin;
  uint8_t _intPin;
  uint8_t previousValue;
  uint8_t currentValue;
  int16_t filteredValue;
  bool isSwitchActive();
  bool isAboveThreshold();
  int16_t getRawValue(MPU6050 &accelgyro);
  uint8_t getMappedMidiValue(int16_t actualValue, int floor = 0, int ceil = 0);
  int16_t runBlockingAverageFilter(int measureSize, MPU6050 &accelgyro, int gap = 1);
  int16_t runNonBlockingAverageFilter();
  int16_t runExponentialFilter(int measureSize, MPU6050 &accelgyro, float alpha = 0.2);
  std::vector< uint8_t > getValuesBetweenRanges(uint8_t gap = 1);
  void setCurrentValue(uint8_t value);
  void setPreviousValue(uint8_t value);
  void setMeasuresCounter(uint8_t value);
  void setDataBuffer(int16_t value);
  void setThreshold(uint8_t value);
  void setMidiMessage(std::string value);
  void sendMidiMessage(BLEMidiServerClass &serverInstance, char messageType[], uint8_t value, const char mode[] = "BLE");
  void setMidiChannel(uint8_t channel);

  static std::vector<Sensor> initializeSensors() {
    std::vector<Sensor> SENSORS = {
      Sensor("potentiometer", 102, A0),
      Sensor("potentiometer", 103, A3),
      Sensor("force", 104, A6),
      Sensor("ax", 105, 0, 18),
      Sensor("ay", 106, 0, 19)
    };
    return SENSORS;
    /**
    * Comment out lines below for STM32 + Xbees support
    * BLE approach will be unavailable for the STM microcontroller
    **/
    //   std::vector<Sensor> SENSORS = {
    //     Sensor("potentiometer", 102, PA0),
    //     Sensor("potentiometer", 103, PA1),
    //     Sensor("force", 104, PA1),
    //     Sensor("ax", 105, 0, PB12),
    //     Sensor("ay", 106, 0, PB14),
    //     Sensor("gx", 105, 0, PB13),
    //     Sensor("gy", 106, 0, PB3),
    //     Sensor("sonar", 107, PB15, PB5)
    //   };
    //   return SENSORS;
  }

  static void setUpSensorPins(std::vector<Sensor> SENSORS) {
    for (Sensor SENSOR : SENSORS) {
      if (!!SENSOR._pin) {
        pinMode(SENSOR._pin, INPUT);
      }
      if (!!SENSOR._intPin) {
        pinMode(SENSOR._intPin, INPUT);
      }
    }
  }
};


#endif