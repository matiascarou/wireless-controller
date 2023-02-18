#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"
#include "Adafruit_VL53L0X.h"

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

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

std::vector<Sensor> SENSORS = Sensor::initializeSensors();

void setup() {
  Serial.begin(115200);
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

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    digitalWrite(ERROR_LED, HIGH);
  }

  Serial.println("Initializing Bluetooth...");

  BLEMidiServer.begin("Le tuts controller");

  // Serial.println("Sensors ready ʕノ•ᴥ•ʔノ");

  Serial.println("Sensors ready (:");
}

unsigned long currentTime = 0;
unsigned long previousTime = 0;

void loop() {
  if (BLEMidiServer.isConnected()) {
    currentTime = millis();
    for (Sensor& SENSOR : SENSORS) {
      if (SENSOR.isSwitchActive()) {
        int16_t rawValue = SENSOR.getRawValue(accelgyro, lox);
        SENSOR.setPreviousRawValue(rawValue);
        SENSOR.setDataBuffer(rawValue);
        if (SENSOR.isAboveThreshold()) {
          const unsigned long currentDebounceValue = millis();
          SENSOR.setCurrentDebounceValue(currentDebounceValue);
          const int16_t averageValue = SENSOR.runNonBlockingAverageFilter();
          const uint8_t sensorMappedValue = SENSOR.getMappedMidiValue(averageValue);
          SENSOR.setPreviousValue(SENSOR.currentValue);
          SENSOR.setCurrentValue(sensorMappedValue);
          SENSOR.debounce(accelgyro, lox);
          SENSOR.sendBleMidiMessage(BLEMidiServer);
          SENSOR.setMeasuresCounter(0);
          SENSOR.setDataBuffer(0);
        }
        SENSOR.setMeasuresCounter(1);
      }
    }
    delay(1);
    // printTotalLoopRuntime(currentTime, previousTime);
  }
}