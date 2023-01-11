#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"

void printMessage(SENSOR sensorInstance) {
  Serial.print(sensorInstance._sensorType);
  Serial.print("::");
  Serial.print(sensorInstance._controllerNumber);
  Serial.print(": ");
  Serial.println(sensorInstance.currentValue);
}

MPU6050 sensor;

const bool DEBUG = false;

SENSOR ANALOG_POTS[] = { SENSOR("analogInput", 102, A0), SENSOR("analogInput", 103, A3) };
SENSOR IMUS[] = { SENSOR("ax", 104, 0, 18), SENSOR("ay", 105, 0, 19) };

void setup() {
  Serial.begin(460800);
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

int measuresBuffer = 1;

const int MAX_NUMBER_OF_MEASURES = 15;

void loop() {
  currentTime = millis();
  if (BLEMidiServer.isConnected()) {
    if (currentTime - previousTime > 10) {
      for (SENSOR& ANALOG_POT : ANALOG_POTS) {
        const int16_t sensorAverageValue = ANALOG_POT.getRawValue(sensor);
        const int sensorMappedValue = ANALOG_POT.getMappedMidiValue(sensorAverageValue, 20, 1023);
        ANALOG_POT.setCurrentValue(sensorMappedValue);
        if (ANALOG_POT.currentValue != ANALOG_POT.previousValue) {
          if (DEBUG) {
            printMessage(ANALOG_POT);
          }
          BLEMidiServer.controlChange(0, ANALOG_POT._controllerNumber, ANALOG_POT.currentValue);
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
        if (IMU.measuresCounter % MAX_NUMBER_OF_MEASURES == 0) {
          IMU.setPreviousValue(IMU.currentValue);
          int16_t averageValue = IMU.getAverageValue(MAX_NUMBER_OF_MEASURES, sensor);
          const int sensorMappedValue = constrain(map(averageValue, 100, 15500, 0, 127), 0, 127);
          IMU.setCurrentValue(sensorMappedValue);
          if (DEBUG) {
            printMessage(IMU);
          }
          // BLEMidiServer.controlChange(0, IMU._controllerNumber, IMU.currentValue);
          std::vector< int > messages = IMU.getValuesBetweenRanges();
          Serial.println(">>>>>>>>>>>>>>>>>");
          for (int message : messages) {            
            BLEMidiServer.controlChange(0, IMU._controllerNumber, message);
            Serial.print("Message: ");
            Serial.println(message);
          }
          Serial.println(">>>>>>>>>>>>>>>>>");
          IMU.measuresCounter = 0;
          IMU.dataBuffer = 0;
        }
        IMU.measuresCounter += 1;
      }
      delayMicroseconds(2000);
    }
  }
}

// Yes, here is an example of a more complex filter called an exponential moving average (EMA) filter:
// #define ALPHA 0.1
// double last_average = 0;

// void loop() {
//   int sensor_reading = analogRead(A0);
//   double average = (sensor_reading * ALPHA) + (last_average * (1.0 - ALPHA));
//   last_average = average;
//   Serial.println(average);
//   delay(1);
// }

// You can also use more advanced filters such as a Kalman filter or a Butterworth filter if you need even more performance. However, these filters can be more complex to implement and may require more computational resources.