// MadgwickFilter.cpp
//
// This file implements the MadgwickFilter class, providing the sensor fusion
// algorithm for attitude estimation. It processes gyroscope and accelerometer
// data to calculate the drone's orientation in terms of quaternions and Euler angles.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "MadgwickFilter.h"
#include <math.h> // Required for sqrtf, atan2f, asinf, fabsf, copysignf

// Constructor initializes the filter with sample frequency and beta.
MadgwickFilter::MadgwickFilter(float sampleFreq, float beta) : _sampleFreq(sampleFreq),
                                                               _beta(beta),
                                                               _q0(1.0f),
                                                               _q1(0.0f),
                                                               _q2(0.0f),
                                                               _q3(0.0f) {}

// Fast inverse square root implementation.
// Used to normalize vectors and quaternions. Uses standard sqrtf for precision.
float MadgwickFilter::_invSqrt(float x)
{
    // Using standard sqrtf for better precision, critical for flight control.
    // This is a deliberate choice over faster, less precise inverse square root approximations.
    return 1.0f / sqrtf(x);
}

// Madgwick filter update function.
// This function performs one step of the Madgwick filter algorithm.
// It takes gyroscope and accelerometer data to estimate orientation.
void MadgwickFilter::update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-_q1 * gx - _q2 * gy - _q3 * gz);
    qDot2 = 0.5f * (_q0 * gx + _q2 * gz - _q3 * gy);
    qDot3 = 0.5f * (_q0 * gy - _q1 * gz + _q3 * gx);
    qDot4 = 0.5f * (_q0 * gz + _q1 * gy - _q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalize accelerometer measurement
        recipNorm = _invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated calculations
        _2q0 = 2.0f * _q0;
        _2q1 = 2.0f * _q1;
        _2q2 = 2.0f * _q2;
        _2q3 = 2.0f * _q3;
        _4q0 = 4.0f * _q0;
        _4q1 = 4.0f * _q1;
        _4q2 = 4.0f * _q2;
        _8q1 = 8.0f * _q1;
        _8q2 = 8.0f * _q2;
        q0q0 = _q0 * _q0;
        q1q1 = _q1 * _q1;
        q2q2 = _q2 * _q2;
        q3q3 = _q3 * _q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * _q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = _4q2 * q3q3 + _2q3 * ay + 4.0f * q0q0 * _q2 - _2q0 * ax - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * _q3 - _2q1 * ax + 4.0f * q2q2 * _q3 - _2q2 * ay;
        recipNorm = _invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // Normalise gradient
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= _beta * s0;
        qDot2 -= _beta * s1;
        qDot3 -= _beta * s2;
        qDot4 -= _beta * s3;
    }

    // Integrate rate of change of quaternion to update quaternion
    _q0 += qDot1 * (1.0f / _sampleFreq);
    _q1 += qDot2 * (1.0f / _sampleFreq);
    _q2 += qDot3 * (1.0f / _sampleFreq);
    _q3 += qDot4 * (1.0f / _sampleFreq);

    // Normalize quaternion
    recipNorm = _invSqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;
}

// Resets the filter's internal quaternion to represent no rotation.
void MadgwickFilter::reset()
{
    _q0 = 1.0f;
    _q1 = 0.0f;
    _q2 = 0.0f;
    _q3 = 0.0f;
}

// Returns the estimated roll angle in degrees.
// Roll is rotation around the X-axis.
float MadgwickFilter::getRoll() const
{
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    // Roll (x-axis rotation)
    return atan2f(_q0 * _q1 + _q2 * _q3, 0.5f - _q1 * _q1 - _q2 * _q2) * (180.0f / M_PI);
}

// Returns the estimated pitch angle in degrees.
// Pitch is rotation around the Y-axis.
float MadgwickFilter::getPitch() const
{
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (_q0 * _q2 - _q1 * _q3);
    if (fabsf(sinp) >= 1.0f) // Use fabsf for float and compare with 1.0f
    {
        return copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI); // Use 90 degrees if out of range
    }
    else
    {
        return asinf(sinp) * (180.0f / M_PI);
    }
}

// Returns the estimated yaw angle in degrees.
// Yaw is rotation around the Z-axis.
float MadgwickFilter::getYaw() const
{
    // Yaw (z-axis rotation)
    return atan2f(_q1 * _q2 + _q0 * _q3, 0.5f - _q2 * _q2 - _q3 * _q3) * (180.0f / M_PI);
}

// Returns the current quaternion components.
void MadgwickFilter::getQuaternion(float &q0, float &q1, float &q2, float &q3) const
{
    q0 = _q0;
    q1 = _q1;
    q2 = _q2;
    q3 = _q3;
}
