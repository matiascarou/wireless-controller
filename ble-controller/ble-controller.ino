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

int previousValue = 0;


void loop() {
  actualValue = millis();
  if (BLEMidiServer.isConnected()) {
    // for (SENSOR& ANALOG_POT : ANALOG_POTS) {
    //   const uint16_t sensorRawData = ANALOG_POT.getRawValue(sensor);
    //   const int sensorMappedValue = ANALOG_POT.getMappedMidiValue(sensorRawData, 50, 1023);
    //   ANALOG_POT.setCurrentValue(sensorMappedValue);
    //   if (ANALOG_POT.currentValue != ANALOG_POT.previousValue) {
    //     Serial.print(ANALOG_POT._sensorType);
    //     Serial.print(": ");
    //     Serial.print(ANALOG_POT.currentValue);
    //     Serial.print('\n');
    //     BLEMidiServer.controlChange(0, ANALOG_POT._controllerNumber, sensorMappedValue);
    //     ANALOG_POT.setPreviousValue(ANALOG_POT.currentValue);
    //   }
    // }

    for (SENSOR& IMU : IMUS) {
      const bool isSensorActive = digitalRead(IMU._intPin);
      if (isSensorActive) {
        uint16_t rawValue = IMU.getAverageValue(20, 1, sensor);
        const int sensorMappedValue = constrain(map(rawValue, 0, 15500, 0, 127), 0, 127);
        IMU.setCurrentValue(sensorMappedValue);
        if (IMU.currentValue != IMU.previousValue) {
          // Serial.print(IMU._sensorType);
          // Serial.print(": ");
          // Serial.print(IMU.currentValue);
          // Serial.print('\n');
          IMU.setPreviousValue(IMU.currentValue);
          BLEMidiServer.controlChange(0, IMU._controllerNumber, IMU.currentValue);
        }
      }
    }
    delay(1);
  }
}