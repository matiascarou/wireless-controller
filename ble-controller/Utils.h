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
static void setAndGetCpuFrequency(const uint32_t& Freq) {
  setCpuFrequencyMhz(Freq);
  const uint32_t esp32ProcessorSpeed = getCpuFrequencyMhz();
  delay(500);
  return esp32ProcessorSpeed;
}
}

#endif