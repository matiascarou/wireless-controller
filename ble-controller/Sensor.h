#ifndef Sensor_h
#define Sensor_h
#include <Arduino.h>
#include "MPU6050.h"
#include "Adafruit_VL53L0X.h"
#include "Wire.h"
#include <vector>
#include <map>
#include "Utils.h"
// #include <BLEMidi.h>

class Sensor {
private:
  uint8_t _controllerNumber;
  char _channel;
  uint8_t _statusCode;
  uint8_t measuresCounter;
  bool isActive;
  bool toggleStatus;
  bool previousToggleStatus;
  bool isAlreadyPressed;
  unsigned long dataBuffer;
  uint16_t _debounceThreshold;
  unsigned long _currentDebounceValue;
  unsigned long _previousDebounceValue;
  unsigned long currentDebounceTimer;
  unsigned long previousDebounceTimer;
  bool previousSwitchState;
  bool currentSwitchState;
  bool isDebounced;
  int16_t _threshold;
  int16_t _floor;
  int16_t _ceil;
  int16_t getInitialValue(std::string &sensorType, std::string valueType);
  int16_t getFloor(std::string &type);
  int16_t getFilterThreshold(std::string &type);
  int16_t getCeil(std::string &type);
  uint16_t getDebounceThreshold(std::string &type);
  uint8_t msb = 0;
  uint8_t lsb = 0;
  uint8_t counter = 0;
public:
  Sensor(const std::string &sensorType, const uint8_t &controllerNumber, const uint8_t &pin = 0, const uint8_t &intPin = 0);
  std::string _sensorType;
  std::string _midiMessage;
  uint8_t filteredExponentialValue;
  uint8_t _pin;
  uint8_t _intPin;
  uint8_t previousValue;
  int16_t previousRawValue;
  uint8_t currentValue;
  int16_t filteredValue;
  bool isSwitchActive();
  bool isAboveThreshold();
  bool isSwitchDebounced();
  int getMappedMidiValue(int16_t actualValue, int floor = 0, int ceil = 0);
  int16_t getRawValue(MPU6050 *accelgyro, Adafruit_VL53L0X *lox);
  int16_t runNonBlockingAverageFilter();
  int16_t runBlockingAverageFilter(int measureSize, MPU6050 *accelgyro, Adafruit_VL53L0X *lox, int gap = 500);
  int16_t runExponentialFilter(MPU6050 *accelgyro, Adafruit_VL53L0X *lox);
  std::vector< uint8_t > getValuesBetweenRanges(uint8_t gap = 1);
  void setCurrentDebounceValue(unsigned long timeValue);
  void setCurrentValue(uint8_t value);
  void setPreviousValue(uint8_t value);
  void setPreviousRawValue(int16_t value);
  void setMeasuresCounter(uint8_t value);
  void setDataBuffer(int16_t value);
  void setThreshold(uint8_t value);
  void setThresholdBasedOnActiveSiblings(uint8_t amountOfActiveSiblings);
  void setMidiMessage(std::string value);
  // void sendBleMidiMessage(BLEMidiServerClass *serverInstance);
  void sendSerialMidiMessage(HardwareSerial *Serial2);
  void setMidiChannel(uint8_t channel);
  void debounce(MPU6050 *accelgyro, Adafruit_VL53L0X *lox);
  std::string getSensorType();

  static void setUpSensorPins(std::vector<Sensor *> SENSORS) {
    for (Sensor *SENSOR : SENSORS) {
      if (!!SENSOR->_pin) {
        pinMode(SENSOR->_pin, INPUT);
      }
      if (!!SENSOR->_intPin) {
        pinMode(SENSOR->_intPin, INPUT);
      }
    }
  }

  /**
  * For ESP32 device.
  **/
  static std::vector<Sensor *> initializeEsp32Sensors() {
    const static std::vector<Sensor *> SENSORS = {
      new Sensor("potentiometer", 102, 15),
      new Sensor("potentiometer", 103, 36),
      new Sensor("force", 104, 4),
      new Sensor("ax", 105, 0, 19),
      new Sensor("ay", 106, 0, 5),
      new Sensor("infrared", 108, 0, 17),
    };
    return SENSORS;
  }

  /**
  * For STM32 device.
  **/
  static std::vector<Sensor *> initializeStm32Sensors() {
    const static std::vector<Sensor *> SENSORS = {
      new Sensor("potentiometer", 102, PA0),
      new Sensor("potentiometer", 103, PA1),
      new Sensor("potentiometer", 104, PA4),
      new Sensor("force", 105, PB1, PB8),
      new Sensor("sonar", 106, PB5, PB15),
      new Sensor("ax", 107, 0, PB12),
      new Sensor("ay", 108, 0, PB14),
    };
    return SENSORS;
  }

  static Sensor *getSensorBySensorType(std::vector<Sensor *> SENSORS, std::string sensorType) {
    for (Sensor *SENSOR : SENSORS) {
      if (SENSOR->_sensorType == sensorType) {
        return SENSOR;
      }
    }
    const std::string errorMessage = "Sensor " + sensorType + " not found";
    Serial.println(errorMessage.c_str());
  }

  static bool isPitchButtonActive(bool &currentButtonState, bool &lastButtonState, bool &toggleStatus, const uint8_t &PITCH_BEND_BUTTON) {
    currentButtonState = !!digitalRead(PITCH_BEND_BUTTON);
    if (currentButtonState && !lastButtonState) {
      toggleStatus = !toggleStatus ? true : false;
    }
    lastButtonState = currentButtonState;
    return toggleStatus;
  }

  static void setInfraredSensorStates(Sensor *infraredSensor, bool &pitchBendLedState, int16_t thresholdValue, std::string midiMessage, bool newLedState, const uint8_t &PITCH_BEND_LED) {
    pitchBendLedState = newLedState;
    digitalWrite(PITCH_BEND_LED, pitchBendLedState);
    infraredSensor->setThreshold(thresholdValue);
    infraredSensor->setMidiMessage(midiMessage);
  }

  static void runPitchBendLogic(Sensor *infraredSensor, const bool &isBendActive, bool &pitchBendLedState, const uint8_t &PITCH_BEND_LED) {
    if (isBendActive && !pitchBendLedState) {
      setInfraredSensorStates(infraredSensor, pitchBendLedState, 2, "pitchBend", true, PITCH_BEND_LED);
    }
    if (!isBendActive && pitchBendLedState) {
      setInfraredSensorStates(infraredSensor, pitchBendLedState, 1, "controlChange", false, PITCH_BEND_LED);
    }
  }

  // static void testAccelgiroConnection(MPU6050 &accelgyro, const uint8_t &ERROR_LED) {
  static void testAccelgiroConnection(MPU6050 &accelgyro) {
    accelgyro.initialize();
    if (accelgyro.testConnection()) {
      Serial.println("Succesfully connected to IMU!");
    } else {
      Serial.println("There was a problem with the IMU initialization");
    }
    delay(100);
  }

  static void testInfraredSensorConnection(Adafruit_VL53L0X &lox, uint8_t i2c_addr, const uint8_t &ERROR_LED, TwoWire *i2c = &Wire) {
    if (!lox.begin(i2c_addr, false, &Wire)) {
      Serial.println("Failed to boot VL53L0X");
      digitalWrite(ERROR_LED, HIGH);
    } else {
      Serial.println("Succesfully connected to VL53L0X!");
    }
    delay(100);
  }

  static void writeSerialMidiMessage(uint8_t statusCode, uint8_t controllerNumber, uint8_t sensorValue, HardwareSerial *Serial2) {
    // Utils::printMidiMessage(statusCode, controllerNumber, sensorValue);
    static const byte rightGuillemet[] = { 0xC2, 0xBB };  //UTF-8 character for separating MIDI messages: 11000010, 10111011
    Serial2->write(char(statusCode));
    Serial2->write(char(controllerNumber));
    Serial2->write(char(sensorValue));
    Serial2->write(rightGuillemet, sizeof(rightGuillemet));
  }

  static bool is_active(Sensor *SENSOR, std::vector<std::string> listOfCandidates) {
    if (std::find(listOfCandidates.begin(), listOfCandidates.end(), SENSOR->_sensorType) != listOfCandidates.end()) {
      return SENSOR->isSwitchActive();
    }
    return false;
  }

  static bool is_debounced(Sensor *SENSOR, std::vector<std::string> listOfCandidates) {
    if (std::find(listOfCandidates.begin(), listOfCandidates.end(), SENSOR->_sensorType) != listOfCandidates.end()) {
      return SENSOR->isSwitchDebounced();
    }
    return false;
  }

  static uint8_t getActiveSiblings(std::vector<Sensor *> SENSORS, std::vector<std::string> candidates) {
    const uint8_t activeSiblings = std::count_if(SENSORS.begin(), SENSORS.end(), [&](Sensor *s) {
      return is_active(s, candidates);
    });
    return activeSiblings;
  }

  static uint8_t areAllSiblingsDebounced(std::vector<Sensor *> SENSORS, std::vector<std::string> candidates) {
    const uint8_t amountOfDebouncedSensors = std::count_if(SENSORS.begin(), SENSORS.end(), [&](Sensor *s) {
      return is_debounced(s, candidates);
    });
    return candidates.size() == amountOfDebouncedSensors;
  }


  // static void writeSerialMidiMessage(uint8_t statusCode, uint8_t controllerNumber, uint8_t sensorValue, HardwareSerial *Serial2) {
  //   uint16_t rightGuillemet = 0xBB00 | 0xC2;  // combine the two bytes into a single uint16_t value
  //   Serial2->write(&statusCode, 1);
  //   Serial2->write(&controllerNumber, 1);
  //   Serial2->write(&sensorValue, 1);
  //   Serial2->write(reinterpret_cast<uint8_t *>(&rightGuillemet), 2);  // reinterpret the uint16_t value as a byte array and send 2 bytes
  // }

  // static void writeSerialMidiMessage(uint8_t statusCode, uint8_t controllerNumber, uint8_t sensorValue, HardwareSerial *Serial2) {
  //   char rightGuillemet[] = u8"\u00BB";
  //   Serial2->write(char(statusCode));
  //   Serial2->write(char(controllerNumber));
  //   Serial2->write(char(sensorValue));
  //   Serial2->write(rightGuillemet);  // reinterpret the uint16_t value as a byte array and send 2 bytes
  // }
};


#endif