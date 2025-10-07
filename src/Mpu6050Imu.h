#ifndef MPU6050_IMU_H
#define MPU6050_IMU_H

#include "ImuInterface.h"
#include <ESP32_MPU6050.h>

// Concrete implementation of ImuInterface for the MPU6050 sensor.
class Mpu6050Imu : public ImuInterface
{
public:
    Mpu6050Imu(TwoWire *wirePort = &Wire);

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
};

#endif // MPU6050_IMU_H
