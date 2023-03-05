#ifndef Sensor_h
#define Sensor_h
#include <Arduino.h>
#include "MPU6050.h"
#include <BLEMidi.h>
#include "Adafruit_VL53L0X.h"
#include "Wire.h"

extern const uint8_t ERROR_LED;

class Sensor {
private:
  char _controllerNumber;
  char _channel;
  char _statusCode;
  uint8_t measuresCounter;
  bool isActive;
  bool toggleStatus;
  bool previousToggleStatus;
  bool isAlreadyPressed;
  unsigned long dataBuffer;
  uint16_t _debounceThreshold;
  unsigned long _currentDebounceValue;
  unsigned long _previousDebounceValue;
  int16_t _threshold;
  int16_t _floor;
  int16_t _ceil;
  int16_t getFloor(std::string &type);
  int16_t getFilterThreshold(std::string &type);
  int16_t getCeil(std::string &type);
  uint16_t getDebounceThreshold(std::string &type);
  uint8_t msb = 0;
  uint8_t lsb = 0;
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
  int getMappedMidiValue(int16_t actualValue, int floor = 0, int ceil = 0);
  int16_t getRawValue(MPU6050 &accelgyro, Adafruit_VL53L0X &lox);
  int16_t runNonBlockingAverageFilter();
  int16_t runBlockingAverageFilter(int measureSize, MPU6050 &accelgyro, Adafruit_VL53L0X &lox, int gap = 500);
  int16_t runExponentialFilter(MPU6050 &accelgyro, Adafruit_VL53L0X &lox);
  std::vector< uint8_t > getValuesBetweenRanges(uint8_t gap = 1);
  void setCurrentDebounceValue(unsigned long timeValue);
  void setCurrentValue(uint8_t value);
  void setPreviousValue(uint8_t value);
  void setPreviousRawValue(int16_t value);
  void setMeasuresCounter(uint8_t value);
  void setDataBuffer(int16_t value);
  void setThreshold(uint8_t value);
  void setMidiMessage(std::string value);
  void sendMidiMessage(BLEMidiServerClass &serverInstance, const char mode[] = "BLE");
  void sendBleMidiMessage(BLEMidiServerClass &serverInstance);
  void sendSerialMidiMessage();
  void setMidiChannel(uint8_t channel);
  void debounce(MPU6050 &accelgyro, Adafruit_VL53L0X &lox);
  std::string getSensorType();

  static Sensor &getSensorBySensorType(std::vector<Sensor> &SENSORS, std::string sensorType) {
    for (Sensor &SENSOR : SENSORS) {
      if (SENSOR._sensorType == sensorType) {
        return SENSOR;
      }
    }
    const std::string errorMessage = "Sensor " + sensorType + " not found";
    Serial.println(errorMessage.c_str());
  }

  static void setUpSensorPins(std::vector<Sensor> &SENSORS) {
    for (Sensor SENSOR : SENSORS) {
      if (!!SENSOR._pin) {
        pinMode(SENSOR._pin, INPUT);
      }
      if (!!SENSOR._intPin) {
        pinMode(SENSOR._intPin, INPUT);
      }
    }
  }

  static std::vector<Sensor> initializeSensors() {
    const static std::vector<Sensor> SENSORS = {
      Sensor("potentiometer", 102, 15),
      Sensor("potentiometer", 103, 36),
      Sensor("force", 104, 4),
      Sensor("ax", 105, 0, 19),
      Sensor("ay", 106, 0, 5),
      Sensor("infrared", 108, 0, 17),
    };
    return SENSORS;
    /**
    * Comment out lines below for STM32 + Xbees support
    * BLE approach will be unavailable for the STM microcontroller
    **/
    //   std::vector<Sensor> SENSORS = {
    //     Sensor("potentiometer", 102, PA0),
    //     Sensor("potentiometer", 103, PA1),
    //     Sensor("force", 104, PA1),
    //     Sensor("ax", 105, 0, PB12),
    //     Sensor("ay", 106, 0, PB14),
    //     Sensor("gx", 105, 0, PB13),
    //     Sensor("gy", 106, 0, PB3),
    //     Sensor("sonar", 107, PB15, PB5)
    //   };
    //   return SENSORS;
  }


  static bool isPitchButtonActive(bool &currentButtonState, bool &lastButtonState, bool &toggleStatus, const uint8_t PITCH_BEND_BUTTON) {
    currentButtonState = !!digitalRead(PITCH_BEND_BUTTON);
    if (currentButtonState && !lastButtonState) {
      toggleStatus = !toggleStatus ? true : false;
    }
    lastButtonState = currentButtonState;
    return toggleStatus;
  }

  static void setInfraredSensorStates(Sensor &infraredSensor, bool &pitchBendLedState, int16_t thresholdValue, std::string midiMessage, bool newLedState, const uint8_t PITCH_BEND_LED) {
    pitchBendLedState = newLedState;
    digitalWrite(PITCH_BEND_LED, pitchBendLedState);
    infraredSensor.setThreshold(thresholdValue);
    infraredSensor.setMidiMessage(midiMessage);
  }

  static void runPitchBendLogic(Sensor &infraredSensor, const bool &isBendActive, bool &pitchBendLedState, const uint8_t PITCH_BEND_LED) {
    if (isBendActive && !pitchBendLedState) {
      setInfraredSensorStates(infraredSensor, pitchBendLedState, 1, "pitchBend", true, PITCH_BEND_LED);
    }
    if (!isBendActive && pitchBendLedState) {
      setInfraredSensorStates(infraredSensor, pitchBendLedState, 2, "controlChange", false, PITCH_BEND_LED);
    }
  }

  static void testAccelgiroConnection(MPU6050 &accelgyro) {
    if (accelgyro.testConnection()) {
      Serial.println("Succesfully connected to IMU!");
    } else {
      Serial.println("There was a problem with the IMU initialization");
      digitalWrite(ERROR_LED, HIGH);
    }
  }

  static void testInfraredSensorConnection(Adafruit_VL53L0X &lox) {
    if (!lox.begin()) {
      Serial.println("Failed to boot VL53L0X");
      digitalWrite(ERROR_LED, HIGH);
    } else {
      Serial.println("Succesfully connected to VL53L0X!");
    }
  }

  static void testI2cSensorsConnection(MPU6050 &accelgyro, Adafruit_VL53L0X &lox) {
    Sensor::testAccelgiroConnection(accelgyro);
    delay(100);
    Sensor::testInfraredSensorConnection(lox);
    delay(100);
  }

  static void checkForI2CDevices(TwoWire &wire) {
    byte error, address;
    int devicesFound = 0;

    Serial.println("Scanning...");

    for (address = 1; address < 127; address++) {
      wire.beginTransmission(address);
      error = wire.endTransmission();

      if (error == 0) {
        Serial.print("Device found at address 0x");
        if (address < 16) {
          Serial.print("0");
        }
        Serial.println(address, HEX);
        devicesFound++;
      }
    }
  }
};


#endif