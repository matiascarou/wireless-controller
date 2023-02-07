// #include "Filter.h"
// #include "Sensor.h"
// #include "MPU6050.h"

// Filter::Filter(Sensor &sensor, MPU6050 &accelgyro)
//   : _sensor(sensor) {
//   _accelgyro = accelgyro;
// }

// int16_t Filter::runBlockingAverageFilter(int measureSize, int gap) {
//   int buffer = 0;
//   for (int i = 0; i < measureSize; i++) {
//     int16_t value = _sensor.getRawValue(_accelgyro);
//     if (value < 0) {
//       value = 0;
//     }
//     buffer += value;
//     delayMicroseconds(gap);
//   }
//   const int16_t result = buffer / measureSize;
//   return result;
// }

// int16_t Filter::runNonBlockingAverageFilter() {
//   return _sensor.dataBuffer / _sensor._threshold;
// }

// int16_t Filter::runExponentialFilter(int measureSize, float alpha) {
//   const int16_t rawValue = _sensor.getRawValue(_accelgyro);
//   _sensor.filteredExponentialValue = (alpha * rawValue) + (1 - alpha) * _sensor.filteredExponentialValue;
//   return _sensor.filteredExponentialValue;
// }