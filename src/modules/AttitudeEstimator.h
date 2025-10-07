#ifndef ATTITUDE_ESTIMATOR_MODULE_H
#define ATTITUDE_ESTIMATOR_MODULE_H

#include "../FlightState.h"
#include <ESP32_MPU6050.h>

// Estimates the drone's attitude using a complementary filter.
//
// This class encapsulates the logic for sensor fusion, taking raw sensor
// data and calculating the roll, pitch, and yaw angles.
class AttitudeEstimator
{
public:
    // Constructor.
    // - imu: A reference to the MPU6050 sensor.
    // - settings: A reference to the flight controller settings.
    AttitudeEstimator(ESP32_MPU6050 &imu, const FlightControllerSettings &settings);

    // Initializes the estimator.
    void begin();

    // Performs one cycle of the attitude calculation.
    // Reads from the IMU and updates the attitude in the FlightState.
    // - state: The current flight state to be updated.
    void update(FlightState &state);

    // Performs sensor calibration by averaging a number of readings.
    // This method is made public to be callable from the FlightController (e.g., via CLI).
    void calibrate();

private:
    ESP32_MPU6050 &_imu; // Reference to the IMU sensor
    const FlightControllerSettings &_settings; // Reference to global settings

    // Sensor offsets
    float _gyroOffsetX = 0.0f;
    float _gyroOffsetY = 0.0f;
    float _gyroOffsetZ = 0.0f;
    float _accelOffsetX = 0.0f;
    float _accelOffsetY = 0.0f;
    float _accelOffsetZ = 0.0f;

    unsigned long _lastUpdateTime = 0;
};

#endif // ATTITUDE_ESTIMATOR_MODULE_H