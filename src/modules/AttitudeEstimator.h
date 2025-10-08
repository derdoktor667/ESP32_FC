#ifndef ATTITUDE_ESTIMATOR_MODULE_H
#define ATTITUDE_ESTIMATOR_MODULE_H

#include "../FlightState.h"
#include "../ImuInterface.h"
#include "MadgwickFilter.h"

// Estimates the drone's attitude using a Madgwick filter.
//
// This class encapsulates the logic for sensor fusion, taking raw sensor
// data and calculating the roll, pitch, and yaw angles.
class AttitudeEstimator
{
public:
    // Default constructor for deferred initialization.
    AttitudeEstimator();

    // Initializes the estimator with references to the IMU and settings.
    void init(ImuInterface &imu, const FlightControllerSettings &settings);

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
    ImuInterface *_imu = nullptr;                        // Pointer to the IMU sensor interface
    const FlightControllerSettings *_settings = nullptr; // Pointer to global settings

    MadgwickFilter _madgwickFilter; // Madgwick filter instance

    unsigned long _lastUpdateTime = 0;
};

#endif // ATTITUDE_ESTIMATOR_MODULE_H