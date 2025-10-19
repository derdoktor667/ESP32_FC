// ComplementaryFilter.cpp
//
// This file implements a simple complementary filter class for attitude estimation.
// It combines gyroscope and accelerometer data to provide robust roll and pitch angles.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "ComplementaryFilter.h"
#include <math.h> // Required for atan2f, asinf

// Constructor: Initializes the filter with a time constant (tau) and sample frequency.
ComplementaryFilter::ComplementaryFilter(float tau, float sampleFreq)
    : _tau(tau),
      _dt(1.0f / sampleFreq),
      _roll(0.0f), _pitch(0.0f), _yaw(0.0f),
      _lastUpdateTime(0)
{
    _alpha = _tau / (_tau + _dt);
}

// Updates the filter with new gyroscope and accelerometer data.
// Returns true if the attitude was updated, false otherwise.
bool ComplementaryFilter::update(float gx, float gy, float gz, float ax, float ay, float az)
{
    // Calculate roll and pitch from accelerometer data
    float accelRoll = atan2f(ay, sqrtf(ax * ax + az * az)) * (180.0f / PI);
    float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / PI);

    // Integrate gyroscope data to get current angles
    unsigned long currentTime = micros();
    float deltaT = (currentTime - _lastUpdateTime) / 1000000.0f; // Convert to seconds
    _lastUpdateTime = currentTime;

    // If deltaT is too large, reset the filter to avoid large jumps
    if (deltaT > 0.1f)
    { // 100ms threshold
        reset();
        return false;
    }

    _roll += gx * deltaT;  // Gyro X (roll) in deg/s
    _pitch += gy * deltaT; // Gyro Y (pitch) in deg/s
    _yaw += gz * deltaT;   // Gyro Z (yaw) in deg/s

    // Apply complementary filter
    _roll = _alpha * (_roll + gx * deltaT) + (1.0f - _alpha) * accelRoll;
    _pitch = _alpha * (_pitch + gy * deltaT) + (1.0f - _alpha) * accelPitch;

    return true;
}

// Resets the filter's internal state.
void ComplementaryFilter::reset()
{
    _roll = 0.0f;
    _pitch = 0.0f;
    _yaw = 0.0f;
    _lastUpdateTime = micros();
}