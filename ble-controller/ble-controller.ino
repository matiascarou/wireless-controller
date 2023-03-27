#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"
#include "Adafruit_VL53L0X.h"
// #include "Utils.h"
// #include <BLEMidi.h>

// const uint8_t ERROR_LED = 2;
// const uint8_t PITCH_BEND_BUTTON = 32;
// const uint8_t PITCH_BEND_LED = 18;

MPU6050 accelgyro;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

std::vector<Sensor*> SENSORS = Sensor::initializeStm32Sensors();
HardwareSerial Serial2(PA3, PA2);  //RX, TX

void setup() {
  delay(500);
  Serial.begin(115200);
  Serial2.begin(230400);
  Serial.println("Starting I2C bus...");
  Wire.begin();
  Wire.setClock(400000L);

  analogReadResolution(10);

  Serial.println("Setting up pins...");

  Sensor::setUpSensorPins(SENSORS);

  // pinMode(ERROR_LED, OUTPUT);
  // pinMode(PITCH_BEND_BUTTON, INPUT);
  // pinMode(PITCH_BEND_LED, OUTPUT);

  Serial.println("Initializing I2C sensors...");

  // Sensor::testInfraredSensorConnection(lox, 0x29, ERROR_LED, &Wire);

  Sensor::testAccelgiroConnection(accelgyro);

  // BLEMidiServer.begin("The performer");

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


void loop() {
  currentTime = millis();
  // const bool isBendActive = Sensor::isPitchButtonActive(currentButtonState, lastButtonState, toggleStatus, PITCH_BEND_BUTTON);
  // Sensor* infraredSensor = Sensor::getSensorBySensorType(SENSORS, "infrared");
  // Sensor::runPitchBendLogic(infraredSensor, isBendActive, pitchBendLedState, PITCH_BEND_LED);
  const uint8_t activeSiblings = Sensor::getActiveSiblings(SENSORS, { "ax", "ay" });
  for (Sensor* SENSOR : SENSORS) {
    if (SENSOR->isSwitchActive()) {
      int16_t rawValue = SENSOR->getRawValue(&accelgyro, &lox);
      SENSOR->setPreviousRawValue(rawValue);
      SENSOR->setDataBuffer(rawValue);
      SENSOR->setMeasuresCounter(1);
      if (SENSOR->isAboveThreshold()) {
        const unsigned long currentDebounceValue = millis();
        SENSOR->setCurrentDebounceValue(currentDebounceValue);
        const int16_t averageValue = SENSOR->runNonBlockingAverageFilter();
        const uint8_t sensorMappedValue = SENSOR->getMappedMidiValue(averageValue);
        SENSOR->setPreviousValue(SENSOR->currentValue);
        SENSOR->setCurrentValue(sensorMappedValue);
        SENSOR->debounce(&accelgyro, &lox);
        SENSOR->sendSerialMidiMessage(&Serial2);
        SENSOR->setMeasuresCounter(0);
        SENSOR->setDataBuffer(0);
        SENSOR->setThresholdBasedOnActiveSiblings(activeSiblings);
      }
    }
  }
  delayMicroseconds(500);
  // Utils::printRuntimeOverrallValue(counter, timeBuffer, previousTime, currentTime);
}