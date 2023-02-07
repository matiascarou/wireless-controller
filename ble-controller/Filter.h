// #ifndef Filter_h
// #define Filter_h
// #include <Arduino.h>
// #include "MPU6050.h"
// #include "Sensor.h"

// class Filter {
// private:
//   MPU6050 _accelgyro;
//   Sensor &_sensor;
// public:
//   Filter(Sensor &sensor, MPU6050 &accelgyro);
//   int16_t runExponentialFilter(int measureSize, float alpha = 0.2);
//   int16_t runBlockingAverageFilter(int measureSize, int gap = 1);
//   int16_t runNonBlockingAverageFilter();
// };


// #endif