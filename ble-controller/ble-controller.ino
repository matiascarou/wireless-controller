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
#define PITCH_BEND_BUTTON 32
#define PITCH_BEND_LED 18

const uint8_t ERROR_LED = 2;

MPU6050 accelgyro;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

std::vector<Sensor> SENSORS = Sensor::initializeSensors();

void setup() {
  delay(100);
  Serial.begin(115200);
  Wire.begin();
  setCpuFrequencyMhz(240);
  // const uint32_t Freq = getCpuFrequencyMhz();
  // Serial.print("CPU Freq is ");
  // Serial.print(Freq);
  // Serial.println(" MHz");
  accelgyro.initialize();

  analogReadResolution(10);

  Sensor::setUpSensorPins(SENSORS);

  pinMode(ERROR_LED, OUTPUT);
  pinMode(PITCH_BEND_BUTTON, INPUT);
  pinMode(PITCH_BEND_LED, OUTPUT);

  Sensor::testAccelgiroConnection(accelgyro);

  delay(100);

  Sensor::testInfraredSensorConnection(lox);

  delay(100);

  Serial.println("Initializing Bluetooth...");

  BLEMidiServer.begin("Le tuts controller");

  delay(100);

  Serial.println("Sensors ready (:");
}

unsigned long currentTime = 0;
unsigned long previousTime = 0;

bool currentButtonState = false;
bool lastButtonState = false;
bool toggleStatus = false;

bool pitchBendLedState = false;

void loop() {
  currentTime = millis();
  if (BLEMidiServer.isConnected()) {
    const bool isBendActive = Sensor::isPitchButtonActive(currentButtonState, lastButtonState, toggleStatus, PITCH_BEND_BUTTON);
    Sensor& infraredSensor = Sensor::getSensorBySensorType(SENSORS, "infrared");
    Sensor::runPitchBendLogic(infraredSensor, isBendActive, pitchBendLedState, PITCH_BEND_LED);
    for (Sensor& SENSOR : SENSORS) {
      if (SENSOR.isSwitchActive()) {
        int16_t rawValue = SENSOR.getRawValue(accelgyro, lox);
        SENSOR.setPreviousRawValue(rawValue);
        SENSOR.setDataBuffer(rawValue);
        if (SENSOR.isAboveThreshold()) {
          const unsigned long currentDebounceValue = millis();
          SENSOR.setCurrentDebounceValue(currentDebounceValue);
          const std::string sensorType = SENSOR.getSensorType();
          int16_t averageValue = sensorType != "infrared" ? SENSOR.runNonBlockingAverageFilter() : SENSOR.runExponentialFilter(accelgyro, lox);
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
  }
  // printTotalLoopRuntime(currentTime, previousTime);
  delay(1);
}

// if (SENSOR._sensorType == "infrared") {
//   Serial.print("sensor raw value: ");
//   Serial.println(rawValue);
// }