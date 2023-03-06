#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"
#include "Adafruit_VL53L0X.h"

void printRuntimeOverrallValue(int& counter, int& timeBuffer, unsigned long& previousTime, unsigned long& currentTime) {
  static const int CYCLES_AMOUNT = 20;
  if (counter % CYCLES_AMOUNT == 0 && counter != 0) {
    Serial.print("Average running time of ");
    Serial.print(CYCLES_AMOUNT);
    Serial.print(" cycles: ");
    Serial.print(timeBuffer);
    Serial.print("\t");
    Serial.print("Average running time of one cycle: ");
    const int diff = timeBuffer / counter;
    Serial.println(diff);
    timeBuffer = 0;
    counter = 0;
  }
  const int timeDiff = currentTime - previousTime;
  previousTime = currentTime;
  timeBuffer += timeDiff;
  counter++;
}

/**
* Code starts here
**/
#define PITCH_BEND_BUTTON 32
#define PITCH_BEND_LED 18

const uint8_t ERROR_LED = 2;

MPU6050 accelgyro;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

std::vector<Sensor*> SENSORS = Sensor::initializeSensorsWithEsp32();

void setup() {
  delay(1000);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000L);
  delay(500);
  setCpuFrequencyMhz(240);
  // const uint32_t Freq = getCpuFrequencyMhz();

  analogReadResolution(10);

  Sensor::setUpSensorPins(SENSORS);

  pinMode(ERROR_LED, OUTPUT);
  pinMode(PITCH_BEND_BUTTON, INPUT);
  pinMode(PITCH_BEND_LED, OUTPUT);

  Sensor::checkForI2CDevices(&Wire);

  Sensor::testInfraredSensorConnection(lox, 0x29, &Wire);

  delay(100);

  accelgyro.initialize();
  Sensor::testAccelgiroConnection(accelgyro);

  delay(100);

  BLEMidiServer.begin("Le tuts controller");

  delay(100);
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
          const std::string sensorType = SENSOR->getSensorType();
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
    // printRuntimeOverrallValue(counter, timeBuffer, previousTime, currentTime);
  }
  delay(1);
}