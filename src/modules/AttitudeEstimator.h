// AttitudeEstimator.h
//
// This file defines the AttitudeEstimator class, responsible for estimating
// the drone's orientation (roll, pitch, yaw) using sensor fusion from IMU data.
// It integrates raw IMU readings with a Madgwick filter for robust attitude estimation.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef ATTITUDE_ESTIMATOR_MODULE_H
#define ATTITUDE_ESTIMATOR_MODULE_MODULE_H

#include "src/config/FlightState.h"
#include "src/hardware/imu/ImuInterface.h"
#include "src/utils/filter/ComplementaryFilter.h"
#include "src/utils/filter/MultiStageBiquadFilter.h"
#include <memory> // Required for std::unique_ptr

// Estimates the drone's attitude (roll, pitch, yaw) using sensor fusion.
// This module encapsulates the logic for reading raw IMU data, applying a
// filter for robust attitude estimation, and updating the FlightState.
// It also handles IMU calibration.
class AttitudeEstimator
{
public:
    // Default constructor.
    // Use init() for proper initialization after settings are loaded.
    AttitudeEstimator();
    // Destructor is no longer explicitly needed as std::unique_ptr handles cleanup

    // Initializes the AttitudeEstimator with required dependencies.
    // This method must be called after settings are loaded and IMU is initialized.
    void init(ImuInterface &imu, const FlightControllerSettings &settings);

    // Performs any necessary setup after initialization.
    // This typically includes starting the filter or other internal components.
    void begin();

    // Updates the drone's attitude based on the latest IMU readings.
    // Reads raw accelerometer and gyroscope data, processes it through the
    // filter, and updates the roll, pitch, and yaw angles in the FlightState.
    void update(FlightState &state);

    // Performs a sensor calibration routine for the IMU.
    // This method delegates the calibration process to the IMU interface.
    void calibrate();

private:
    ImuInterface *_imu = nullptr;                        // Pointer to the IMU sensor interface
    const FlightControllerSettings *_settings = nullptr; // Pointer to global flight controller settings

    std::unique_ptr<ComplementaryFilter> _complementaryFilter; // Instance of the Complementary filter for sensor fusion

    // Multi-stage biquad low-pass filters for raw gyroscope data
    std::unique_ptr<MultiStageBiquadFilter> _gyroRollLpf;
    std::unique_ptr<MultiStageBiquadFilter> _gyroPitchLpf;
    std::unique_ptr<MultiStageBiquadFilter> _gyroYawLpf;

    // Multi-stage biquad notch filters for raw gyroscope data
    std::unique_ptr<MultiStageBiquadFilter> _gyroRollNotchFilter;
    std::unique_ptr<MultiStageBiquadFilter> _gyroPitchNotchFilter;
    std::unique_ptr<MultiStageBiquadFilter> _gyroYawNotchFilter;

    // Multi-stage biquad low-pass filters for raw accelerometer data
    std::unique_ptr<MultiStageBiquadFilter> _accelRollLpf;
    std::unique_ptr<MultiStageBiquadFilter> _accelPitchLpf;
    std::unique_ptr<MultiStageBiquadFilter> _accelYawLpf;

    // Private helper method for filter initialization
    void _initializeFilters();


};

#endif // ATTITUDE_ESTIMATOR_MODULE_H