#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"
#include <iostream>
#include <algorithm>

MPU6050 sensor;

SENSOR ANALOG_POTS[] = { SENSOR("analogInput", 102, A0), SENSOR("analogInput", 103, A3) };
SENSOR IMUS[] = { SENSOR("ax", 105, 0, 18), SENSOR("ay", 106, 0, 19) };
SENSOR sonar = SENSOR("sonar", 107, 13);

void setup() {
  Serial.begin(230400);
  Wire.begin();
  Serial.println(F("Initializing bluetooth"));
  sensor.initialize();

  if (sensor.testConnection()) {
    Serial.println(F("Succesfully connected to IMU!"));
  } else {
    Serial.println(F("There was a problem with the IMU initialization"));
  }

  analogReadResolution(10);

  for (SENSOR ANALOG_POT : ANALOG_POTS) {
    pinMode(ANALOG_POT._pin, INPUT);
  }

  for (SENSOR IMU : IMUS) {
    pinMode(IMU._intPin, INPUT);
  }

  pinMode(sonar._pin, INPUT);

  BLEMidiServer.begin("Le tuts controller");
}

unsigned long previousTime = 0;
unsigned long currentTime = 0;

const int IMU_MAX_NUMBER_OF_MEASURES = 15;
const int SONAR_MAX_NUMBER_OF_MEASURES = 3;

void loop() {
  currentTime = millis();
  if (BLEMidiServer.isConnected()) {

    // const auto sonarRawValue = sonar.getRawValue(sensor);
    // sonar.dataBuffer += sonarRawValue;
    // if (sonar.measuresCounter % SONAR_MAX_NUMBER_OF_MEASURES == 0) {
    //   sonar.currentValue = sonar.getAverageValue(SONAR_MAX_NUMBER_OF_MEASURES, sensor);
    //   if (sonar.previousValue != sonar.currentValue) {
    //     if (DEBUG) {
    //       printMessage(sonar);
    //     }
    //     BLEMidiServer.controlChange(0, sonar._controllerNumber, sonar.currentValue);
    //     sonar.previousValue = sonar.currentValue;
    //   }
    //   sonar.measuresCounter = 0;
    //   sonar.dataBuffer = 0;
    // }
    // sonar.measuresCounter += 1;

    if (currentTime - previousTime > 10) {
      for (SENSOR& ANALOG_POT : ANALOG_POTS) {
        const int16_t sensorAverageValue = ANALOG_POT.getRawValue(sensor);
        const int sensorMappedValue = ANALOG_POT.getMappedMidiValue(sensorAverageValue, 20, 1023);
        ANALOG_POT.setCurrentValue(sensorMappedValue);
        if (ANALOG_POT.currentValue != ANALOG_POT.previousValue) {
          ANALOG_POT.sendMidiMessage(BLEMidiServer, "controlChange", ANALOG_POT.currentValue);
          ANALOG_POT.setPreviousValue(ANALOG_POT.currentValue);
        }
      }
      previousTime = currentTime;
    }

    for (SENSOR& IMU : IMUS) {
      const bool isSensorActive = digitalRead(IMU._intPin);
      if (isSensorActive) {
        int16_t rawValue = IMU.getRawValue(sensor);
        int16_t normalizedRawValue = constrain(rawValue, 0, 32767);
        IMU.dataBuffer += normalizedRawValue;
        if (IMU.measuresCounter % IMU_MAX_NUMBER_OF_MEASURES == 0) {
          IMU.setPreviousValue(IMU.currentValue);
          int16_t averageValue = IMU.runNonBlockingAverageFilter(IMU_MAX_NUMBER_OF_MEASURES);
          const int sensorMappedValue = constrain(map(averageValue, 100, 15500, 0, 127), 0, 127);
          IMU.setCurrentValue(sensorMappedValue);
          std::vector< int > messages = IMU.getValuesBetweenRanges();
          for (int message : messages) {
            IMU.sendMidiMessage(BLEMidiServer, "controlChange", message);
          }
          IMU.measuresCounter = 0;
          IMU.dataBuffer = 0;
        }
        IMU.measuresCounter += 1;
      }
    }
    delay(1);
  }
}