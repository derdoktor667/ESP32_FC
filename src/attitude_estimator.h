#ifndef ATTITUDE_ESTIMATOR_H
#define ATTITUDE_ESTIMATOR_H

#include <Arduino.h>
#include <ESP32_MPU6050.h>
#include "config.h"          // For COMPLEMENTARY_FILTER_GAIN
#include "mpu_calibration.h" // For gyro and accel offsets

extern ESP32_MPU6050 imuSensor; // Declare external MPU object

extern float roll, pitch, yaw;                  // Declare external attitude variables
extern unsigned long last_attitude_update_time; // External last attitude update time

void calculateAttitude();

#endif
