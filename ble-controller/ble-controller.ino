#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"
#include "Adafruit_VL53L0X.h"
// #include <BLEMidi.h>
// #include "Utils.h"

// const uint8_t ERROR_LED = 2;
// const uint8_t PITCH_BEND_BUTTON = 32;
// const uint8_t PITCH_BEND_LED = 18;

MPU6050 accelgyro;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// std::vector<Sensor*> SENSORS = Sensor::initializeEsp32Sensors();
std::vector<Sensor*> SENSORS = Sensor::initializeStm32Sensors();
HardwareSerial Serial2(PA3, PA2);  //RX, TX

void setup() {
  delay(500);
  // const uint32_t Freq = Utils::setAndGetEsp32CpuFrequency(240);
  Serial.begin(230400);
  Serial2.begin(230400);
  Serial.println("Starting I2C bus...");
  Wire.begin();
  // Wire.setClock(200000);
  // delay(500);

  analogReadResolution(10);

  Serial.println("Setting up pins...");

  Sensor::setUpSensorPins(SENSORS);

  // pinMode(ERROR_LED, OUTPUT);
  // pinMode(PITCH_BEND_BUTTON, INPUT);
  // pinMode(PITCH_BEND_LED, OUTPUT);

  // Utils::checkForI2CDevices(&Wire);

  Serial.println("Initializing I2C sensors...");

  // Sensor::testInfraredSensorConnection(lox, 0x29, ERROR_LED, &Wire);

  Sensor::testAccelgiroConnection(accelgyro);

  // BLEMidiServer.begin("The performer");

  // delay(500);

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
  // if (BLEMidiServer.isConnected()) {
  currentTime = millis();
  // const bool isBendActive = Sensor::isPitchButtonActive(currentButtonState, lastButtonState, toggleStatus, PITCH_BEND_BUTTON);
  // Sensor* infraredSensor = Sensor::getSensorBySensorType(SENSORS, "infrared");
  // Sensor::runPitchBendLogic(infraredSensor, isBendActive, pitchBendLedState, PITCH_BEND_LED);
  for (Sensor* SENSOR : SENSORS) {
    if (SENSOR->isSwitchActive()) {
      SENSOR->setMeasuresCounter(1);
      int16_t rawValue = SENSOR->getRawValue(&accelgyro, &lox);
      SENSOR->setPreviousRawValue(rawValue);
      SENSOR->setDataBuffer(rawValue);
      if (SENSOR->isAboveThreshold()) {
        const unsigned long currentDebounceValue = millis();
        SENSOR->setCurrentDebounceValue(currentDebounceValue);
        int16_t averageValue = SENSOR->runNonBlockingAverageFilter();
        const uint8_t sensorMappedValue = SENSOR->getMappedMidiValue(averageValue);
        SENSOR->setPreviousValue(SENSOR->currentValue);
        SENSOR->setCurrentValue(sensorMappedValue);
        SENSOR->debounce(&accelgyro, &lox);
        // SENSOR->sendBleMidiMessage(&BLEMidiServer);
        SENSOR->sendSerialMidiMessage(&Serial2);
        SENSOR->setMeasuresCounter(0);
        SENSOR->setDataBuffer(0);
        const uint8_t activeParents = Sensor::getActiveParents(SENSORS);
        SENSOR->setActiveParents(activeParents);
      }
    }
  }
  // Utils::printRuntimeOverrallValue(counter, timeBuffer, previousTime, currentTime);
  // }
  delay(1);
}