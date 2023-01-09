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

int measuresBuffer = 1;

const int MAX_NUMBER_OF_MEASURES = 40;

void loop() {
  currentTime = millis();
  if (BLEMidiServer.isConnected()) {
    if (currentTime - previousTime > 10) {
      for (SENSOR& ANALOG_POT : ANALOG_POTS) {
        const int16_t sensorAverageValue = ANALOG_POT.getRawValue(sensor);
        const int sensorMappedValue = ANALOG_POT.getMappedMidiValue(sensorAverageValue, 50, 900);
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
        int16_t rawValue = IMU.getRawValue(sensor);
        if (rawValue < 0) {
          rawValue = 0;
        }
        // Serial.print("Raw value: ");
        // Serial.println(rawValue);
        IMU.dataBuffer += rawValue;
        if (IMU.measuresCounter % MAX_NUMBER_OF_MEASURES == 0) {
          // Serial.print('\t');
          // Serial.print("Measures counter: ");
          // Serial.print(IMU.measuresCounter);
          // Serial.print('\t');
          // Serial.print("Data buffer: ");
          // Serial.print(IMU.dataBuffer);
          // Serial.print("\t");
          int16_t averageValue = IMU.getAverageValue(MAX_NUMBER_OF_MEASURES);
          // Serial.print("Average value: ");
          // Serial.println(averageValue);
          const int sensorMappedValue = constrain(map(averageValue, 50, 15000, 0, 127), 0, 127);
          IMU.setCurrentValue(sensorMappedValue);
          if (IMU.currentValue != IMU.previousValue) {
            printMessage(IMU);
            BLEMidiServer.controlChange(0, IMU._controllerNumber, IMU.currentValue);
            IMU.setPreviousValue(IMU.currentValue);
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

void printMessage(SENSOR sensorInstance) {
  Serial.print(sensorInstance._sensorType);
  Serial.print("::");
  Serial.print(sensorInstance._controllerNumber);
  Serial.print(": ");
  Serial.print(sensorInstance.currentValue);
  Serial.print('\n');
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