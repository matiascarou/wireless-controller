#include <Arduino.h>
#include <BLEMidi.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "Sensor.h"

int potPins[] = { A0, A3 };
int potValues[] = { 0, 0 };
int potsPrevValue[] = { 0, 0 };

unsigned long prevValue = 0;
unsigned long actualValue = 0;

MPU6050 sensor;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  Serial.begin(230400);
  Wire.begin();
  Serial.println("Initializing bluetooth");
  sensor.initialize();
  if (sensor.testConnection()) {
    Serial.println("Succesfully connected to IMU!");
  } else {
    Serial.println("There was a problem with the IMU initialization");
  }
  BLEMidiServer.begin("El controller del tuts");

  for (int potPin : potPins) {
    pinMode(potPin, INPUT);
  }
  analogReadResolution(10);
}

int potentiometers() {
  if (actualValue - prevValue > 40) {
    for (int i = 0; i < 2; i++) {
      potValues[i] = constrain(map(analogRead(potPins[i]), 0, 1023, 0, 127), 0, 127);
      if (potsPrevValue[i] != potValues[i]) {
        BLEMidiServer.controlChange(0, (102 + i), potValues[i]);
        potsPrevValue[i] = potValues[i];
      }
    }
    prevValue = actualValue;
  }
}

SENSOR mpu = SENSOR("IMU", 0);

void loop() {
  actualValue = millis();
  if (BLEMidiServer.isConnected()) {
    //ACCELEROMETER
    ax = mpu.getRawValue(sensor);
    ax = constrain(map(ax, 0, 16500, 0, 127), 0, 127);
    BLEMidiServer.controlChange(0, 102, ax);
    //POTENTIOMETERS
    // for (int i = 0; i < 2; i++) {
    //   potValues[i] = constrain(map(analogRead(potPins[i]), 0, 1023, 0, 127), 0, 127);
    //   if (potsPrevValue[i] != potValues[i]) {
    //     BLEMidiServer.controlChange(0, (102 + i), potValues[i]);
    //     potsPrevValue[i] = potValues[i];
    //   }
    // }
  }
  delay(10);
}