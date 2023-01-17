// Sketch uses 1097769 bytes (83%) of program storage space.
//  1096797
#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"

MPU6050 sensor;

SENSOR ANALOG_POTS[] = { SENSOR("analogInput", 102, A0), SENSOR("analogInput", 103, A3) };
SENSOR IMUS[] = { SENSOR("ax", 105, 0, 18), SENSOR("ay", 106, 0, 19) };
SENSOR sonar = SENSOR("sonar", 107, 13);

void printMessage(uint8_t message) {
  Serial.print("Message: ");
  Serial.print(message);
  Serial.print("\n");
}

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

  pinMode(sonar._pin, INPUT);

  BLEMidiServer.begin("Le tuts controller");

  Serial.println("Sensors ready ʕノ•ᴥ•ʔノ");
}

unsigned long previousTime = 0;
unsigned long currentTime = 0;

const uint8_t IMU_MAX_NUMBER_OF_MEASURES = 10;
const uint8_t SONAR_MAX_NUMBER_OF_MEASURES = 3;

void loop() {
  currentTime = millis();
  if (BLEMidiServer.isConnected()) {
    if (currentTime - previousTime > 10) {
      for (SENSOR& ANALOG_POT : ANALOG_POTS) {
        const int16_t sensorAverageValue = ANALOG_POT.getRawValue(sensor);
        const uint8_t sensorMappedValue = ANALOG_POT.getMappedMidiValue(sensorAverageValue, 20, 1023);
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
          const int16_t averageValue = IMU.runNonBlockingAverageFilter(IMU_MAX_NUMBER_OF_MEASURES);
          const uint8_t sensorMappedValue = constrain(map(averageValue, 100, 15500, 0, 127), 0, 127);
          IMU.setCurrentValue(sensorMappedValue);
          std::vector< uint8_t > messages = IMU.getValuesBetweenRanges();
          for (uint8_t message : messages) {
            IMU.sendMidiMessage(BLEMidiServer, "controlChange", message);
          }
          IMU.measuresCounter = 0;
          IMU.dataBuffer = 0;
        }
        IMU.measuresCounter += 1;
      }
    }
    delayMicroseconds(500);
  }
}

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