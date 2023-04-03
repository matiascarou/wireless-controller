#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"
#include "Adafruit_VL53L0X.h"
// #include "Utils.h"

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

  // Sensor::testInfraredSensorConnection(lox, 0x29, ERROR_LED, &Wire);

  Sensor::testAccelgiroConnection(accelgyro);

  Serial.println("System ready <(':'<)");
}

unsigned long currentTime = 0;
unsigned long previousTime = 0;
int timeBuffer = 0;
int counter = 0;

bool currentButtonState = false;
bool lastButtonState = false;
bool toggleStatus = false;
bool pitchBendLedState = false;

const static std::vector<std::string> SIBLINGS = { "ax", "ay" };

void loop() {
  currentTime = millis();
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
  // Utils::printRuntimeOverrallValue(counter, timeBuffer, previousTime, currentTime, 20);
}