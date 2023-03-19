#ifndef UTILS_H
#define UTILS_H

namespace Utils {
static void printRuntimeOverrallValue(int& counter, int& timeBuffer, unsigned long& previousTime, unsigned long& currentTime) {
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
static void setAndGetEsp32CpuFrequency(const uint32_t& Freq) {
  setCpuFrequencyMhz(Freq);
  const uint32_t esp32ProcessorSpeed = getCpuFrequencyMhz();
  delay(500);
  return esp32ProcessorSpeed;
}
static void checkForI2CDevices(TwoWire* wire) {
  byte error, address;
  int devicesFound = 0;

  Serial.println("Scanning I2C bus...");

  for (address = 1; address < 127; address++) {
    wire->beginTransmission(address);
    error = wire->endTransmission();

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
}

#endif