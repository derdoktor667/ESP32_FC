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
#include <Wire.h> // Required for I2C communication

// Constructor: Initializes the MPU6050 object.
Mpu6050Imu::Mpu6050Imu(LpfBandwidth lpfBandwidth, ImuRotation imuRotation)
    : _mpu(), _lpfBandwidth(lpfBandwidth), _imuRotation(imuRotation) // Default constructor for the underlying MPU6050 library object
{
}

// Initializes the MPU6050 sensor.
bool Mpu6050Imu::begin()
{
    Wire.begin();                      // Initialize I2C communication
    Wire.setClock(I2C_CLOCK_SPEED_HZ); // Set I2C clock speed to 400kHz

    // Using highest ranges for gyroscope (2000 DPS) and accelerometer (16G)
    // to ensure full measurement capability for a flight controller.
    return _mpu.begin(GYRO_RANGE_2000DPS, ACCEL_RANGE_16G, _lpfBandwidth);
}

// Updates the raw sensor data from the MPU6050.
void Mpu6050Imu::update()
{
    _mpu.update();

    // Apply IMU rotation based on settings
    float tempAccelX = _mpu.readings.accelerometer.x;
    float tempAccelY = _mpu.readings.accelerometer.y;
    float tempAccelZ = _mpu.readings.accelerometer.z;

    float tempGyroX = _mpu.readings.gyroscope.x;
    float tempGyroY = _mpu.readings.gyroscope.y;
    float tempGyroZ = _mpu.readings.gyroscope.z;

    switch (_imuRotation)
    {
    case IMU_ROTATION_NONE:
        // No rotation needed
        break;
    case IMU_ROTATION_90_DEG_CW:
        _mpu.readings.accelerometer.x = tempAccelY;
        _mpu.readings.accelerometer.y = -tempAccelX;
        _mpu.readings.gyroscope.x = tempGyroY;
        _mpu.readings.gyroscope.y = -tempGyroX;
        break;
    case IMU_ROTATION_180_DEG_CW:
        _mpu.readings.accelerometer.x = -tempAccelX;
        _mpu.readings.accelerometer.y = -tempAccelY;
        _mpu.readings.gyroscope.x = -tempGyroX;
        _mpu.readings.gyroscope.y = -tempGyroY;
        break;
    case IMU_ROTATION_270_DEG_CW:
        _mpu.readings.accelerometer.x = -tempAccelY;
        _mpu.readings.accelerometer.y = tempAccelX;
        _mpu.readings.gyroscope.x = -tempGyroY;
        _mpu.readings.gyroscope.y = tempGyroX;
        break;
    case IMU_ROTATION_FLIP:
        _mpu.readings.accelerometer.y = -tempAccelY;
        _mpu.readings.accelerometer.z = -tempAccelZ;
        _mpu.readings.gyroscope.y = -tempGyroY;
        _mpu.readings.gyroscope.z = -tempGyroZ;
        break;
    default:
        // Should not happen, but handle gracefully
        break;
    }
}

// Performs MPU6050-specific calibration.
bool Mpu6050Imu::calibrate(int numReadings)
{
    _mpu.calibrate(numReadings);
    return true; // MPU6050 library calibration always returns true if it runs
}
