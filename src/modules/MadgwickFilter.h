#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <Arduino.h> // Required for float and other Arduino types

// Madgwick filter class for attitude estimation using accelerometer and gyroscope data.
class MadgwickFilter
{
public:
    // Constructor for MadgwickFilter.
    // @param sampleFreq The sampling frequency of the IMU data in Hz.
    // @param beta The Madgwick filter gain parameter. A higher beta means more trust in accelerometer data.
    MadgwickFilter(float sampleFreq, float beta);

    // Updates the filter with new sensor data.
    // @param gx Gyroscope x-axis reading in rad/s.
    // @param gy Gyroscope y-axis reading in rad/s.
    // @param gz Gyroscope z-axis reading in rad/s.
    // @param ax Accelerometer x-axis reading in g's.
    // @param ay Accelerometer y-axis reading in g's.
    // @param az Accelerometer z-axis reading in g's.
    void update(float gx, float gy, float gz, float ax, float ay, float az);

    // Returns the estimated roll angle in degrees.
    float getRoll() const;

    // Returns the estimated pitch angle in degrees.
    float getPitch() const;

    // Returns the estimated yaw angle in degrees.
    float getYaw() const;

    // Returns the current quaternion components.
    // @param q0 Reference to store the w component of the quaternion.
    // @param q1 Reference to store the x component of the quaternion.
    // @param q2 Reference to store the y component of the quaternion.
    // @param q3 Reference to store the z component of the quaternion.
    void getQuaternion(float &q0, float &q1, float &q2, float &q3) const;

private:
    float _sampleFreq; // Sample frequency in Hz
    float _beta;       // Madgwick filter gain

    // Quaternion components
    volatile float _q0;
    volatile float _q1;
    volatile float _q2;
    volatile float _q3;

    // Fast inverse square root
    float _invSqrt(float x);
};

#endif // MADGWICK_FILTER_H
