#ifndef SENSOR_h
#define SENSOR_h
#include <Arduino.h>
#include "MPU6050.h"

class SENSOR {
private:
  int pin;
  unsigned long previousValue;
  unsigned long actualValue;
  char sensorType[];
public:
  SENSOR(char sensorType[], int pin = 0);
  int getRawValue(MPU6050 sensor);
  static int getAverageValue(unsigned long value, int measureSize, int gap);
  // int getAverageValue(unsigned long value, int measureSize, int gap);
  // int turnON();
};

#endif