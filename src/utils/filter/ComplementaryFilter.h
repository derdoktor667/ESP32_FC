// ComplementaryFilter.h
//
// This file defines a simple complementary filter class for attitude estimation.
// It combines gyroscope and accelerometer data to provide robust roll and pitch angles.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <Arduino.h>

class ComplementaryFilter
{
public:
    // Constructor: Initializes the filter with a time constant (tau) and sample frequency.
    ComplementaryFilter(float tau, float sampleFreq);

    // Updates the filter with new gyroscope and accelerometer data.
    // Returns true if the attitude was updated, false otherwise.
    bool update(float gx, float gy, float gz, float ax, float ay, float az);

    // Returns the estimated roll angle in degrees.
    float getRoll() const { return _roll; }

    // Returns the estimated pitch angle in degrees.
    float getPitch() const { return _pitch; }

    // Returns the estimated yaw angle in degrees (from gyroscope integration only).
    float getYaw() const { return _yaw; }

    // Resets the filter's internal state.
    void reset();

private:
    float _tau;   // Time constant for the filter
    float _alpha; // Filter coefficient derived from tau and sample frequency
    float _dt;    // Time step (1/sampleFreq)

    float _roll, _pitch, _yaw; // Estimated angles

    unsigned long _lastUpdateTime; // Last time the filter was updated

    static constexpr float MAX_DELTA_T_THRESHOLD = 0.1f; // Threshold for deltaT to prevent large jumps
};

#endif // COMPLEMENTARY_FILTER_H