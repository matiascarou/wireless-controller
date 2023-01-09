#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"

MPU6050 sensor;

SENSOR ANALOG_POTS[] = { SENSOR("analogInput", 102, A0), SENSOR("analogInput", 103, A3) };
SENSOR IMUS[] = { SENSOR("ax", 104, 0, 18), SENSOR("ay", 105, 0, 19) };

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

  for (SENSOR IMU : IMUS) {
    pinMode(IMU._intPin, INPUT);
  }

  BLEMidiServer.begin("El controller del tuts");
}

unsigned long previousTime = 0;
unsigned long currentTime = 0;

void loop() {

  currentTime = millis();
  // if (BLEMidiServer.isConnected()) {
  if (currentTime - previousTime > 10) {
    for (SENSOR& ANALOG_POT : ANALOG_POTS) {
      const int16_t sensorAverageValue = ANALOG_POT.getRawValue(sensor);
      const int sensorMappedValue = ANALOG_POT.getMappedMidiValue(sensorAverageValue, 50, 1000);
      ANALOG_POT.setCurrentValue(sensorMappedValue);
      if (ANALOG_POT.currentValue != ANALOG_POT.previousValue) {
        printMessage(ANALOG_POT);
        BLEMidiServer.controlChange(0, ANALOG_POT._controllerNumber, ANALOG_POT.currentValue);
        ANALOG_POT.setPreviousValue(ANALOG_POT.currentValue);
      }
    }
    previousTime = currentTime;
  }

  for (SENSOR& IMU : IMUS) {
    const bool isSensorActive = digitalRead(IMU._intPin);
    if (isSensorActive) {
      uint16_t rawValue = IMU.getAverageValue(15, 500, sensor);
      const int sensorMappedValue = constrain(map(rawValue, 0, 15500, 0, 127), 0, 127);
      IMU.setCurrentValue(sensorMappedValue);
      if (IMU.currentValue != IMU.previousValue) {
        printMessage(IMU);
        BLEMidiServer.controlChange(0, IMU._controllerNumber, IMU.currentValue);
        IMU.setPreviousValue(IMU.currentValue);
      }
    }
  }
  delay(1);
  // }
}

void printMessage(SENSOR sensorInstance) {
  Serial.print(sensorInstance._sensorType);
  Serial.print("::");
  Serial.print(sensorInstance._controllerNumber);
  Serial.print(": ");
  Serial.print(sensorInstance.currentValue);
  Serial.print('\n');
}