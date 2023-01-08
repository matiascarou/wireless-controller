#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"

unsigned long prevValue = 0;
unsigned long actualValue = 0;

MPU6050 sensor;

SENSOR ANALOG_POTS[] = { SENSOR("analogInput", 102, A0), SENSOR("analogInput", 103, A3) };
SENSOR IMUS[] = { SENSOR("ax", 104, 0), SENSOR("ay", 105, 0) };

void setup() {
  Serial.begin(230400);
  Wire.begin();
  Serial.println("Initializing bluetooth");
  sensor.initialize();

  if (sensor.testConnection()) {
    Serial.println("Succesfully connected to IMU!");
  } else {
    Serial.println("There was a problem with the IMU initialization");
  }

  analogReadResolution(10);

  for (SENSOR ANALOG_POT : ANALOG_POTS) {
    pinMode(ANALOG_POT._pin, INPUT);
  }

  // BLEMidiServer.begin("El controller del tuts");
}

void loop() {
  actualValue = millis();
  // if (BLEMidiServer.isConnected()) {
  for (SENSOR ANALOG_POT : ANALOG_POTS) {
    const uint16_t sensorRawData = ANALOG_POT.getRawValue(sensor);
    const int sensorMappedValue = ANALOG_POT.getMappedMidiValue(sensorRawData, 50, 1023);
    Serial.print("controller_number ");
    Serial.print(ANALOG_POT._controllerNumber);
    Serial.print(": ");
    Serial.print(sensorMappedValue);
    Serial.print("\t");
  }
  Serial.print("\n");
  // for (SENSOR imu : IMUS) {
  //   int16_t axisData = imu.getRawValue(sensor);
  //   const int sensorMappedValue = imu.getMappedMidiValue(axisData, 500, 16000);
  //   Serial.print("Axis value for the axis ");
  //   Serial.print(imu._sensorType);
  //   Serial.print(": ");
  //   Serial.println(axisData);
  //   BLEMidiServer.controlChange(0, imu._controllerNumber, sensorMappedValue);
  // }
  delay(40);
  // }
}