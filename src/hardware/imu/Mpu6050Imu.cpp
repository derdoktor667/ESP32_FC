// Mpu6050Imu.cpp
//
// This file implements the Mpu6050Imu class, providing the concrete functionality
// for interacting with the MPU6050 IMU sensor. It uses the ESP32_MPU6050 library
// to read accelerometer, gyroscope, and temperature data.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "src/hardware/imu/Mpu6050Imu.h"

// Constructor: Initializes the MPU6050 object.
Mpu6050Imu::Mpu6050Imu(LpfBandwidth lpfBandwidth)
    : _mpu(), _lpfBandwidth(lpfBandwidth) // Default constructor for the underlying MPU6050 library object
{
}

// Initializes the MPU6050 sensor.
bool Mpu6050Imu::begin()
{
    // Using highest ranges for gyroscope (2000 DPS) and accelerometer (16G)
    // to ensure full measurement capability for a flight controller.
    return _mpu.begin(GYRO_RANGE_2000DPS, ACCEL_RANGE_16G, _lpfBandwidth);
}

// Updates the raw sensor data from the MPU6050.
void Mpu6050Imu::update()
{
    _mpu.update();
}

// Performs MPU6050-specific calibration.
bool Mpu6050Imu::calibrate(int numReadings)
{
    Serial.println("INFO: Calibrating MPU6050... Keep the drone still.");
    _mpu.calibrate(numReadings);
    Serial.println("INFO: MPU6050 Calibration complete.");
    return true; // MPU6050 library calibration always returns true if it runs
}
