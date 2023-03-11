#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"
#include "Adafruit_VL53L0X.h"
// #include "Utils.h"

const uint8_t ERROR_LED = 2;
const uint8_t PITCH_BEND_BUTTON = 32;
const uint8_t PITCH_BEND_LED = 18;

MPU6050 accelgyro;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

std::vector<Sensor*> SENSORS = Sensor::initializeEsp32Sensors();

void setup() {
  delay(1000);
  // const uint32_t Freq = Utils::setAndGetCpuFrequency(240);
  Serial.begin(115200);
  Serial.println("Starting I2C bus...");
  Wire.begin();
  delay(500);
  // Wire.setClock(200000);
  // delay(500);

  analogReadResolution(10);

  Serial.println("Setting up pins...");

  Sensor::setUpSensorPins(SENSORS);

  pinMode(ERROR_LED, OUTPUT);
  pinMode(PITCH_BEND_BUTTON, INPUT);
  pinMode(PITCH_BEND_LED, OUTPUT);

  // Sensor::checkForI2CDevices(&Wire);

  Serial.println("Initializing I2C sensors...");

  Sensor::testInfraredSensorConnection(lox, 0x29, ERROR_LED, &Wire);

  delay(200);

  accelgyro.initialize();
  Sensor::testAccelgiroConnection(accelgyro, ERROR_LED);

  delay(200);

  Serial.println("Initializing BLE server...");

  BLEMidiServer.begin("The performer");

  delay(500);
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
  if (BLEMidiServer.isConnected()) {
    currentTime = millis();
    const bool isBendActive = Sensor::isPitchButtonActive(currentButtonState, lastButtonState, toggleStatus, PITCH_BEND_BUTTON);
    Sensor* infraredSensor = Sensor::getSensorBySensorType(SENSORS, "infrared");
    Sensor::runPitchBendLogic(infraredSensor, isBendActive, pitchBendLedState, PITCH_BEND_LED);
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
          SENSOR->sendBleMidiMessage(&BLEMidiServer);
          SENSOR->setMeasuresCounter(0);
          SENSOR->setDataBuffer(0);
        }
      }
    }
    // Utils::printRuntimeOverrallValue(counter, timeBuffer, previousTime, currentTime);
  }
  delay(1);
}