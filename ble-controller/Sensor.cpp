// LED.cpp
#include "Sensor.h"
#include "MPU6050.h"

SENSOR::SENSOR(char sensorType[], int pin) {
  pin = pin;
  previousValue = 0;
  actualValue = 0;
  sensorType = sensorType;
  // ledState = LOW;
  // pinMode(ledPin, OUTPUT);
}

int SENSOR::getAverageValue(unsigned long value, int measureSize, int gap) {
  int buffer = 0;
  for (int i = 0; i < measureSize; i++) {
    value = constrain(map(value, 0, 16500, 0, 127), 0, 127);
    buffer += value;
    delay(gap);
  }
  return buffer / measureSize;
}

int SENSOR::getRawValue(MPU6050 sensor) {

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  if (String(sensorType).equals(String("analogInput")) == true) {
    return analogRead(pin);
  }

  if (String(sensorType).equals(String("IMU")) == true) {
    Serial.println("ENTEREDDDDDD");
    sensor.getAcceleration(&ax, &ay, &az);
    return ax;
  }

  sensor.getAcceleration(&ax, &ay, &az);
  return ax;

  return 0;
}