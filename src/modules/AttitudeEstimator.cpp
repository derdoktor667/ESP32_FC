#include "AttitudeEstimator.h"
#include "../config.h"
#include <Arduino.h>

// Constructor: Initializes with references to the IMU and settings.
AttitudeEstimator::AttitudeEstimator(ESP32_MPU6050 &imu, const FlightControllerSettings &settings)
    : _imu(imu), _settings(settings) // Initialize references
{
}

// Initializes the estimator and performs the initial calibration.
void AttitudeEstimator::begin()
{
    Serial.println("Initializing MPU6050...");
    if (!_imu.begin())
    {
        Serial.println("Failed to find MPU6050 chip. Halting.");
        while (1) { delay(10); }
    }
    Serial.println("MPU6050 initialized.");

    calibrate();
    _lastUpdateTime = micros();
}

// Performs the main attitude calculation.
void AttitudeEstimator::update(FlightState &state)
{
    unsigned long currentTime = micros();
    float dt = (currentTime - _lastUpdateTime) / 1000000.0f; // Delta time in seconds
    _lastUpdateTime = currentTime;

    _imu.update(); // Read the latest sensor data

    // Apply calibration offsets to raw sensor data
    state.gyroRates.roll = _imu.readings.gyroscope.x - _gyroOffsetX;
    state.gyroRates.pitch = _imu.readings.gyroscope.y - _gyroOffsetY;
    state.gyroRates.yaw = _imu.readings.gyroscope.z - _gyroOffsetZ;
    state.accelReadings.roll = _imu.readings.accelerometer.x - _accelOffsetX;
    state.accelReadings.pitch = _imu.readings.accelerometer.y - _accelOffsetY;
    state.accelReadings.yaw = _imu.readings.accelerometer.z - _accelOffsetZ; // Using yaw component of Vector3f for Z

    // --- Complementary Filter ---
    // Calculate roll and pitch angles from the accelerometer
    float accelRoll = atan2(state.accelReadings.pitch, state.accelReadings.yaw) * 180 / PI;
    float accelPitch = atan2(-state.accelReadings.roll, sqrt(state.accelReadings.pitch * state.accelReadings.pitch + state.accelReadings.yaw * state.accelReadings.yaw)) * 180 / PI;

    // Fuse gyro and accel data
    float gain = _settings.filter.complementaryFilterGain;
    state.attitude.roll = gain * (state.attitude.roll + state.gyroRates.roll * dt) + (1.0f - gain) * accelRoll;
    state.attitude.pitch = gain * (state.attitude.pitch + state.gyroRates.pitch * dt) + (1.0f - gain) * accelPitch;
    state.attitude.yaw += state.gyroRates.yaw * dt; // Yaw is integrated from gyro only
}

// Performs sensor calibration by averaging a number of readings.
void AttitudeEstimator::calibrate()
{
    Serial.println("Calibrating IMU. Keep the drone level and still...");
    int numReadings = _settings.calibration.mpuCalibrationReadings;
    Vector3f gyroSum, accelSum;

    for (int i = 0; i < numReadings; i++)
    {
        _imu.update();
        gyroSum.roll += _imu.readings.gyroscope.x;
        gyroSum.pitch += _imu.readings.gyroscope.y;
        gyroSum.yaw += _imu.readings.gyroscope.z;
        accelSum.roll += _imu.readings.accelerometer.x;
        accelSum.pitch += _imu.readings.accelerometer.y;
        accelSum.yaw += _imu.readings.accelerometer.z;
        delay(1);
    }

    // Calculate average offsets
    _gyroOffsetX = gyroSum.roll / numReadings;
    _gyroOffsetY = gyroSum.pitch / numReadings;
    _gyroOffsetZ = gyroSum.yaw / numReadings;
    _accelOffsetX = accelSum.roll / numReadings;
    _accelOffsetY = accelSum.pitch / numReadings;
    _accelOffsetZ = (accelSum.yaw / numReadings) - _settings.calibration.accelZGravity;

    Serial.println("IMU calibration complete.");
}
