#ifndef SENSOR_h
#define SENSOR_h
#include <Arduino.h>
#include "MPU6050.h"

class SENSOR {
  // private:
  //   unsigned long previousValue;
  //   unsigned long currentValue;
public:
  SENSOR(const char* sensorType, const int controllerNumber, uint8_t pin = 0, uint8_t int_pin = 0);
  const char* _sensorType;
  int _controllerNumber;
  uint8_t _pin;
  uint8_t _intPin;
  int previousValue;
  int currentValue;
  int16_t getRawValue(MPU6050 sensor);
  int getMappedMidiValue(int16_t actualValue, int floor, int ceil);
  int16_t getAverageValue(int measureSize, float gap, MPU6050 sensor);
  void setCurrentValue(int value);
  void setPreviousValue(int value);
};

#endif