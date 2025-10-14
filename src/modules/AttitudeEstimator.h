#ifndef ATTITUDE_ESTIMATOR_MODULE_H
#define ATTITUDE_ESTIMATOR_MODULE_H

#include "src/config/FlightState.h"
#include "src/hardware/imu/ImuInterface.h"
#include "src/utils/filter/MadgwickFilter.h"

// Estimates the drone's attitude (roll, pitch, yaw) using sensor fusion.
// This module encapsulates the logic for reading raw IMU data, applying a
// Madgwick filter for robust attitude estimation, and updating the FlightState.
// It also handles IMU calibration.
class AttitudeEstimator
{
public:
    // Default constructor.
    // Use init() for proper initialization after settings are loaded.
    AttitudeEstimator();

    // Initializes the AttitudeEstimator with required dependencies.
    // This method must be called after settings are loaded and IMU is initialized.
    void init(ImuInterface &imu, const FlightControllerSettings &settings);

    // Performs any necessary setup after initialization.
    // This typically includes starting the Madgwick filter or other internal components.
    void begin();

    // Updates the drone's attitude based on the latest IMU readings.
    // Reads raw accelerometer and gyroscope data, processes it through the
    // Madgwick filter, and updates the roll, pitch, and yaw angles in the FlightState.
    void update(FlightState &state);

    // Performs a sensor calibration routine for the IMU.
    // This method averages a number of IMU readings to determine sensor biases.
    // It is typically triggered via the CLI.
    void calibrate();

private:
    ImuInterface *_imu = nullptr;                        // Pointer to the IMU sensor interface
    const FlightControllerSettings *_settings = nullptr; // Pointer to global flight controller settings

    MadgwickFilter _madgwickFilter; // Instance of the Madgwick filter for sensor fusion


};

#endif // ATTITUDE_ESTIMATOR_MODULE_H