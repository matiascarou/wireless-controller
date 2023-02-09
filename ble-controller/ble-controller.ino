#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"

void printTotalLoopRuntime(unsigned long current, unsigned long& previous) {
  Serial.print("Loop total time: ");
  Serial.println(current - previous);
  previous = current;
}

/**
* Code starts here
**/
#define ERROR_LED 2

MPU6050 accelgyro;

std::vector<Sensor> SENSORS = Sensor::initializeSensors();

void setup() {
  Serial.begin(230400);
  Wire.begin();
  delay(500);
  Serial.println("Initializing IMU...");
  accelgyro.initialize();

  analogReadResolution(10);
  Sensor::setUpSensorPins(SENSORS);
  pinMode(ERROR_LED, OUTPUT);

  if (accelgyro.testConnection()) {
    Serial.println("Succesfully connected to IMU!");
  } else {
    Serial.println("There was a problem with the IMU initialization");
    digitalWrite(ERROR_LED, HIGH);
  }

  Serial.println("Initializing Bluetooth...");

  BLEMidiServer.begin("Le tuts controller");

  Serial.println("Sensors ready ʕノ•ᴥ•ʔノ");
}

void loop() {
  if (BLEMidiServer.isConnected()) {
    for (Sensor& SENSOR : SENSORS) {
      if (SENSOR.isSwitchActive()) {
        int16_t rawValue = SENSOR.getRawValue(accelgyro);
        SENSOR.setDataBuffer(rawValue);
        if (SENSOR.isAboveThreshold()) {
          const unsigned long currentDebounceValue = millis();
          SENSOR.setCurrentDebounceValue(currentDebounceValue);
          const int16_t averageValue = SENSOR.runNonBlockingAverageFilter();
          const uint8_t sensorMappedValue = SENSOR.getMappedMidiValue(averageValue);
          SENSOR.setPreviousValue(SENSOR.currentValue);
          SENSOR.setCurrentValue(sensorMappedValue);
          SENSOR.debounce(accelgyro);
          SENSOR.sendBleMidiMessage(BLEMidiServer);
          SENSOR.setMeasuresCounter(0);
          SENSOR.setDataBuffer(0);
        }
        SENSOR.setMeasuresCounter(1);
      }
    }
  }
  delayMicroseconds(500);
  // printTotalLoopRuntime(currentTime, previousTime);
}