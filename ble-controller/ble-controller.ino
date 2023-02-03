#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"

MPU6050 accelgyro;

std::vector<Sensor> SENSORS = Sensor::initializeSensors();

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
  accelgyro.initialize();

  if (accelgyro.testConnection()) {
    Serial.println("Succesfully connected to IMU!");
  } else {
    Serial.println("There was a problem with the IMU initialization");
  }

  analogReadResolution(10);

  Sensor::setUpSensorPins(SENSORS);

  // for (Sensor SENSOR : SENSORS) {
  //   if (!!SENSOR._pin) {
  //     pinMode(SENSOR._pin, INPUT);
  //   }
  //   if (!!SENSOR._intPin) {
  //     pinMode(SENSOR._intPin, INPUT);
  //   }
  // }


  BLEMidiServer.begin("Le tuts controller");

  Serial.println("Sensors ready ʕノ•ᴥ•ʔノ");
}

unsigned long previousTime = 0;
unsigned long currentTime = 0;

void loop() {
  currentTime = millis();
  if (BLEMidiServer.isConnected()) {
    for (Sensor& SENSOR : SENSORS) {
      if (SENSOR.isSwitchActive()) {
        int16_t rawValue = SENSOR.getRawValue(accelgyro);
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