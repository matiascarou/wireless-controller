#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"

#define ERROR_LED 2

MPU6050 accelgyro;

std::vector<Sensor> SENSORS = Sensor::initializeSensors();

void printMessage(uint8_t message) {
  Serial.print("Message: ");
  Serial.print(message);
  Serial.print("\n");
}

void printTotalLoopRuntime(unsigned long current, unsigned long& previous) {
  Serial.print("Loop total time: ");
  Serial.println(current - previous);
  previous = current;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Initializing bluetooth");
  accelgyro.initialize();
  pinMode(ERROR_LED, OUTPUT);

  if (accelgyro.testConnection()) {
    Serial.println("Succesfully connected to IMU!");
  } else {
    Serial.println("There was a problem with the IMU initialization");
    digitalWrite(ERROR_LED, HIGH);
  }

  analogReadResolution(10);

  Sensor::setUpSensorPins(SENSORS);

  BLEMidiServer.begin("Le tuts controller");

  // Serial.println("Sensors ready ʕノ•ᴥ•ʔノ");
  Serial.println("Sensors ready (:");
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
  }
}