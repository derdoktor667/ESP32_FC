// Mpu6050Imu.h
//
// This file defines the Mpu6050Imu class, a concrete implementation of the
// ImuInterface for the MPU6050 sensor. It acts as an adapter for the
// ESP32_MPU6050 library, providing a standardized way to interact with the IMU.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef MPU6050_IMU_H
#define MPU6050_IMU_H

#include "src/hardware/imu/ImuInterface.h"
#include "src/config/config.h" // Include config.h for ImuRotation
#include <ESP32_MPU6050.h>

// Concrete implementation of ImuInterface for the MPU6050 sensor.
class Mpu6050Imu : public ImuInterface
{
public:
    Mpu6050Imu(LpfBandwidth lpfBandwidth, ImuRotation imuRotation);

    bool begin() override;
    void update() override;
    bool calibrate(int numReadings) override;

    float getAccelX() const override { return _mpu.readings.accelerometer.x; }
    float getAccelY() const override { return _mpu.readings.accelerometer.y; }
    float getAccelZ() const override { return _mpu.readings.accelerometer.z; }

    float getGyroRollRate() const override { return _mpu.readings.gyroscope.x; }
    float getGyroPitchRate() const override { return _mpu.readings.gyroscope.y; }
    float getGyroYawRate() const override { return _mpu.readings.gyroscope.z; }

    float getTemperature() const override { return _mpu.readings.temperature_celsius; }

private:
    ESP32_MPU6050 _mpu;
    LpfBandwidth _lpfBandwidth;
    ImuRotation _imuRotation; // New member to store IMU rotation

    // Private helper method for logging calibration status
    void _logCalibrationStatus(const char *message);
};

#endif // MPU6050_IMU_H
