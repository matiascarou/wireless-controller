#ifndef SENSOR_h
#define SENSOR_h
#include <Arduino.h>
#include "MPU6050.h"

class SENSOR {
private:
  unsigned long previousValue;
  unsigned long actualValue;
public:
  SENSOR(const char *sensorType, const int controllerNumber, uint8_t pin = 0);
  uint16_t getRawValue(MPU6050 sensor);
  int getMappedMidiValue(int16_t actualValue, int floor, int ceil);
  static int getAverageValue(unsigned long value, int measureSize, int gap);
  const char *_sensorType;
  int _controllerNumber;
  uint8_t _pin;
};

#endif