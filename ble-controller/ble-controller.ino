#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"

MPU6050 sensor;

/**
* This script is intended to work both with BLE and Xbee protocols
* The serial approach will need of an Xbee receptor to be previously hooked up
* And listening to the coordinator's output
**/
const char COMMUNICATION_TYPE[] = "BLE";

// Sensor ANALOG_POTS[] = { Sensor("analogInput", 102, A0), Sensor("analogInput", 103, A3) };
// Sensor IMUS[] = { Sensor("ax", 105, 0, 18), Sensor("ay", 106, 0, 19) };
// Sensor sonar = Sensor("sonar", 107, 13);

Sensor SENSORS[] = { Sensor("analogInput", 102, A0), Sensor("analogInput", 103, A3), Sensor("ax", 105, 0, 18), Sensor("ay", 106, 0, 19) };

/**
* Comment out lines below for STM32 + Xbees support
* BLE approach will not work with the STM microcontroller
**/
// ANALOG_POTS = { Sensor("analogInput", 102, PA0), Sensor("analogInput", 103, PA1), Sensor("analogInput", 103, PB1) };
// IMUS = { Sensor("ax", 105, 0, PB12), Sensor("ay", 106, 0, PB14), Sensor("gx", 105, 0, PB13), Sensor("gy", 106, 0, PB3) };
// sonar = Sensor("sonar", 107, PB15, PB5);

void printMessage(uint8_t message) {
  Serial.print("Message: ");
  Serial.print(message);
  Serial.print("\n");
}

void printTotalLoopRuntime(unsigned long current, unsigned long previous) {
  Serial.print("Loop total time: ");
  Serial.println(current - previous);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Initializing bluetooth");
  sensor.initialize();

  if (sensor.testConnection()) {
    Serial.println("Succesfully connected to IMU!");
  } else {
    Serial.println("There was a problem with the IMU initialization");
  }

  analogReadResolution(10);

  // for (Sensor ANALOG_POT : ANALOG_POTS) {
  //   pinMode(ANALOG_POT._pin, INPUT);
  // }

  // for (Sensor IMU : IMUS) {
  //   pinMode(IMU._intPin, INPUT);
  // }

  // pinMode(sonar._pin, INPUT);

  for (Sensor SENSOR : SENSORS) {
    if (SENSOR._pin) {
      pinMode(SENSOR._pin, INPUT);
    }
    if (SENSOR._intPin) {
      pinMode(SENSOR._intPin, INPUT);
    }
  }


  BLEMidiServer.begin("Le tuts controller");

  Serial.println("Sensors ready ʕノ•ᴥ•ʔノ");
}

unsigned long previousTime = 0;
unsigned long currentTime = 0;

void loop() {
  currentTime = millis();
  if (BLEMidiServer.isConnected()) {
    for (Sensor& SENSOR : SENSORS) {
      if (SENSOR.isActive()) {
        int16_t rawValue = SENSOR.getRawValue(sensor);
        SENSOR.setDataBuffer(rawValue);
        if (SENSOR.isAboveThreshold()) {
          const int16_t averageValue = SENSOR.runNonBlockingAverageFilter();
          const uint8_t sensorMappedValue = SENSOR.getMappedMidiValue(averageValue);
          SENSOR.setPreviousValue(SENSOR.currentValue);
          SENSOR.setCurrentValue(sensorMappedValue);
          if (SENSOR.currentValue != SENSOR.previousValue) {
            SENSOR.sendMidiMessage(BLEMidiServer, "controlChange", SENSOR.currentValue);
          }
          SENSOR.setMeasuresCounter(0);
          SENSOR.setDataBuffer(0);
        }
        SENSOR.setMeasuresCounter(1);
      }
    }
    delayMicroseconds(500);
    printTotalLoopRuntime(currentTime, previousTime);
    previousTime = currentTime;
  }
}

/**
* OLD APPROACH
**/

// const uint8_t IMU_MAX_NUMBER_OF_MEASURES = 40;
// const uint8_t SONAR_MAX_NUMBER_OF_MEASURES = 3;

// if (currentTime - previousTime > 10) {
//   for (Sensor& ANALOG_POT : ANALOG_POTS) {
//     const int16_t sensorAverageValue = ANALOG_POT.getRawValue(sensor);
//     const uint8_t sensorMappedValue = ANALOG_POT.getMappedMidiValue(sensorAverageValue, 20, 1023);
//     ANALOG_POT.setCurrentValue(sensorMappedValue);
//     if (ANALOG_POT.currentValue != ANALOG_POT.previousValue) {
//       ANALOG_POT.sendMidiMessage(BLEMidiServer, "controlChange", ANALOG_POT.currentValue);
//       ANALOG_POT.setPreviousValue(ANALOG_POT.currentValue);
//     }
//   }
//   previousTime = currentTime;
// }

// for (Sensor& IMU : IMUS) {
//   if (IMU.isActive()) {
//     int16_t rawValue = IMU.getRawValue(sensor);
//     int16_t normalizedRawValue = constrain(rawValue, 0, 15500);
//     IMU.setDataBuffer(normalizedRawValue);
//     if (IMU.measuresCounter % IMU_MAX_NUMBER_OF_MEASURES == 0) {
//       const int16_t averageValue = IMU.runNonBlockingAverageFilter(IMU_MAX_NUMBER_OF_MEASURES);
//       const uint8_t sensorMappedValue = IMU.getMappedMidiValue(averageValue, 100, 15500);
//       IMU.setPreviousValue(IMU.currentValue);
//       IMU.setCurrentValue(sensorMappedValue);
//       if (IMU.currentValue != IMU.previousValue) {
//         IMU.sendMidiMessage(BLEMidiServer, "controlChange", IMU.currentValue);
//       }
//       IMU.setMeasuresCounter(0);
//       IMU.setDataBuffer(0);
//     }
//     IMU.setMeasuresCounter(1);
//   }
// }
/**
* END OLD APPROACH
**/

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