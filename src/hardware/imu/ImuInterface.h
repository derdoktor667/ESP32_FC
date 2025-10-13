#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <Arduino.h>

// Abstract base class for IMU sensors.
// Defines a common interface for different IMU hardware.
class ImuInterface
{
public:
    virtual ~ImuInterface() = default;

    // Initializes the IMU sensor.
    // Returns true on successful initialization, false otherwise.
    virtual bool begin() = 0;

    // Updates the raw sensor data from the IMU.
    virtual void update() = 0;

    // Performs sensor-specific calibration.
    // Returns true on successful calibration, false otherwise.
    virtual bool calibrate(int numReadings) = 0;

    // Retrieves accelerometer data.
    virtual float getAccelX() const = 0;
    virtual float getAccelY() const = 0;
    virtual float getAccelZ() const = 0;

    // Retrieves gyroscope data (rates).
    virtual float getGyroRollRate() const = 0;  // Degrees/second
    virtual float getGyroPitchRate() const = 0; // Degrees/second
    virtual float getGyroYawRate() const = 0;   // Degrees/second

    // Retrieves temperature (optional, but common).
    virtual float getTemperature() const = 0;
};

#endif // IMU_INTERFACE_H
