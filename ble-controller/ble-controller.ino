#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"
#include "Adafruit_VL53L0X.h"

MPU6050 accelgyro;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

std::vector<Sensor*> SENSORS = Sensor::initializeStm32Sensors();
HardwareSerial Serial2(PA3, PA2);  //RX, TX

void setup() {
  delay(250);
  Serial.begin(230400);
  Serial2.begin(230400);
  Serial.println("Starting I2C bus...");
  Wire.begin();

  analogReadResolution(10);

  Serial.println("Setting up pins...");

  Sensor::setUpSensorPins(SENSORS);

  Serial.println("Initializing I2C sensors...");

  Sensor::testAccelgiroConnection(accelgyro);

  Serial.println("System ready <(':'<)");
}

const static std::vector<std::string> SIBLINGS = { "ax", "ay" };

void loop() {
  const uint8_t activeSiblings = Sensor::getActiveSiblings(SENSORS, SIBLINGS);
  const uint8_t areAllSiblingsDebounced = Sensor::areAllSiblingsDebounced(SENSORS, SIBLINGS);
  for (Sensor* SENSOR : SENSORS) {
    const bool isSiblingButWithPendingDebounce = SENSOR->isSibling(SIBLINGS) && !areAllSiblingsDebounced;
    if (isSiblingButWithPendingDebounce || !SENSOR->isSwitchActive()) {
      continue;
    }
    SENSOR->run(&accelgyro, &lox, activeSiblings);
  }
  delay(1);
}