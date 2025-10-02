#ifndef MPU_CALIBRATION_H
#define MPU_CALIBRATION_H

#include <Arduino.h>
#include <ESP32_MPU6050.h>
#include "config.h"

extern ESP32_MPU6050 mpu; // External MPU object

extern float gyro_offset_x, gyro_offset_y, gyro_offset_z; // External gyro offsets
extern float acc_offset_x, acc_offset_y, acc_offset_z;    // External accel offsets

void calibrateMPU6050();

#endif
