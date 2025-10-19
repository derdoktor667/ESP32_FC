// AttitudeEstimator.cpp
//
// This file implements the AttitudeEstimator class, which is responsible for
// processing IMU data and estimating the drone's attitude (roll, pitch, yaw).
// It utilizes a Complementary filter for sensor fusion and delegates IMU-specific
// calibration to the IMU interface.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "src/modules/AttitudeEstimator.h"
#include "src/config/config.h"
#include <Arduino.h>

// Default constructor: Initializes pointers to nullptr and Madgwick filter with dummy values.
// Actual initialization happens in init() once settings are available.
AttitudeEstimator::AttitudeEstimator()
    : _imu(nullptr), _settings(nullptr),
      _gyroRollNotchFilter(nullptr), _gyroPitchNotchFilter(nullptr), _gyroYawNotchFilter(nullptr)
{
}

// Init method: Sets up references to IMU and settings, then properly initializes the filter.
// This method must be called after settings are loaded and IMU is initialized.
void AttitudeEstimator::init(ImuInterface &imu, const FlightControllerSettings &settings)
{
    _imu = &imu;
    _settings = &settings;
    _initializeFilters();
}

// Performs the initial calibration of the IMU.
void AttitudeEstimator::begin()
{
    // Delegate initial calibration to the IMU interface.!
    calibrate();
}

// Performs one cycle of attitude calculation: reads IMU, updates filter, and stores attitude in state.
void AttitudeEstimator::update(FlightState &state)
{
    if (!_imu)
    {
        Serial.println("ERROR: IMU interface not initialized in AttitudeEstimator.");
        return;
    }

    _imu->update(); // Read the latest raw sensor data from the IMU

    // Get raw gyroscope data
    float rawGx = _imu->getGyroRollRate();
    float rawGy = _imu->getGyroPitchRate();
    float rawGz = _imu->getGyroYawRate();

    // Apply biquad low-pass filter to gyroscope data
    float gx = _gyroRollLpf->update(rawGx);
    float gy = _gyroPitchLpf->update(rawGy);
    float gz = _gyroYawLpf->update(rawGz);

    // Apply gyroscope notch filters if enabled
    if (_settings->filter.enableGyroNotchFilter && _gyroRollNotchFilter)
    {
        gx = _gyroRollNotchFilter->update(gx);
        gy = _gyroPitchNotchFilter->update(gy);
        gz = _gyroYawNotchFilter->update(gz);
    }

    // Get raw accelerometer data
    float rawAx = _imu->getAccelX();
    float rawAy = _imu->getAccelY();
    float rawAz = _imu->getAccelZ();

    // Apply biquad low-pass filter to accelerometer data
    float ax = _accelRollLpf->update(rawAx);
    float ay = _accelPitchLpf->update(rawAy);
    float az = _accelYawLpf->update(rawAz);

    // Update the Complementary filter with the latest sensor readings.
    // This performs sensor fusion to estimate orientation.
    _complementaryFilter->update(gx, gy, gz, ax, ay, az);

    // Retrieve the calculated attitude (roll, pitch, yaw) from the Complementary filter
    // and store it in the FlightState.
    state.attitude.roll = _complementaryFilter->getRoll();
    state.attitude.pitch = _complementaryFilter->getPitch();
    state.attitude.yaw = _complementaryFilter->getYaw();
}

// Performs sensor calibration by averaging a number of readings.
// This method delegates the calibration process to the IMU interface.
void AttitudeEstimator::calibrate()
{
    _imu->calibrate(_settings->calibration.mpuCalibrationReadings);
    // After IMU calibration, reset the filter to consider the current
    // orientation as the new horizontal zero position.
    _complementaryFilter->reset();
}

void AttitudeEstimator::updateNotchFilterState()
{
    if (_settings->filter.enableGyroNotchFilter)
    {
        // Create notch filters if they don't exist
        if (!_gyroRollNotchFilter)
        {
            _gyroRollNotchFilter = std::make_unique<MultiStageBiquadFilter>(_settings->filter.gyroNotchFreq, _settings->filter.filterSampleFreq, _settings->filter.gyroNotchQ, NOTCH);
            _gyroPitchNotchFilter = std::make_unique<MultiStageBiquadFilter>(_settings->filter.gyroNotchFreq, _settings->filter.filterSampleFreq, _settings->filter.gyroNotchQ, NOTCH);
            _gyroYawNotchFilter = std::make_unique<MultiStageBiquadFilter>(_settings->filter.gyroNotchFreq, _settings->filter.filterSampleFreq, _settings->filter.gyroNotchQ, NOTCH);
        }
    }
    else
    {
        // Destroy notch filters if they exist
        _gyroRollNotchFilter.reset();
        _gyroPitchNotchFilter.reset();
        _gyroYawNotchFilter.reset();
    }
}

void AttitudeEstimator::_initializeFilters()
{
    // Initialize Complementary filter with actual settings from config.h
    _complementaryFilter = std::make_unique<ComplementaryFilter>(_settings->filter.complementaryFilterTau, _settings->filter.filterSampleFreq);

    // Initialize gyroscope low-pass filters
    _gyroRollLpf = std::make_unique<MultiStageBiquadFilter>(_settings->filter.gyroLpfCutoffFreq, _settings->filter.filterSampleFreq, _settings->filter.gyroLpfStages);
    _gyroPitchLpf = std::make_unique<MultiStageBiquadFilter>(_settings->filter.gyroLpfCutoffFreq, _settings->filter.filterSampleFreq, _settings->filter.gyroLpfStages);
    _gyroYawLpf = std::make_unique<MultiStageBiquadFilter>(_settings->filter.gyroLpfCutoffFreq, _settings->filter.filterSampleFreq, _settings->filter.gyroLpfStages);

    // Initialize gyroscope notch filters if enabled (at boot)
    if (_settings->filter.enableGyroNotchFilter)
    {
        updateNotchFilterState(); // Call the new function to initialize them
    }

    // Initialize accelerometer low-pass filters
    _accelRollLpf = std::make_unique<MultiStageBiquadFilter>(_settings->filter.accelLpfCutoffFreq, _settings->filter.filterSampleFreq, _settings->filter.accelLpfStages);
    _accelPitchLpf = std::make_unique<MultiStageBiquadFilter>(_settings->filter.accelLpfCutoffFreq, _settings->filter.filterSampleFreq, _settings->filter.accelLpfStages);
    _accelYawLpf = std::make_unique<MultiStageBiquadFilter>(_settings->filter.accelLpfCutoffFreq, _settings->filter.filterSampleFreq, _settings->filter.accelLpfStages);
}
