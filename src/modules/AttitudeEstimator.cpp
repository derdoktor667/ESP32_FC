#include "AttitudeEstimator.h"
#include "../config.h"
#include <Arduino.h>

// Default constructor: Initializes pointers to nullptr.
AttitudeEstimator::AttitudeEstimator()
    : _imu(nullptr), _settings(nullptr), // Initialize pointers to nullptr
      _madgwickFilter(0.0f, 0.0f) // Initialize Madgwick filter with dummy values
{
}

// Init method: Sets up references and initializes the Madgwick filter.
void AttitudeEstimator::init(ImuInterface &imu, const FlightControllerSettings &settings)
{
    _imu = &imu;
    _settings = &settings;
    // Re-initialize Madgwick filter with actual settings
    _madgwickFilter = MadgwickFilter(_settings->filter.madgwickSampleFreq, _settings->filter.madgwickBeta);
}

// Initializes the estimator and performs the initial calibration.
void AttitudeEstimator::begin()
{
    Serial.println("Initializing IMU...");
    if (!_imu->begin())
    {
        Serial.println("Failed to initialize IMU. Halting.");
        while (1) { delay(IMU_INIT_FAIL_DELAY_MS); }
    }
    Serial.println("IMU initialized.");

    calibrate();
    _lastUpdateTime = micros();
}

// Performs the main attitude calculation.
void AttitudeEstimator::update(FlightState &state)
{
    // The Madgwick filter internally handles the time step, but we still need to update _lastUpdateTime
    // for potential future uses or for debugging.
    unsigned long currentTime = micros();
    _lastUpdateTime = currentTime;

    _imu->update(); // Read the latest sensor data

    // Get calibrated sensor data from the IMU interface
    // Gyroscope data should be in rad/s
    // Accelerometer data should be in g's
    float gx = radians(_imu->getGyroRollRate());
    float gy = radians(_imu->getGyroPitchRate());
    float gz = radians(_imu->getGyroYawRate());
    float ax = _imu->getAccelX();
    float ay = _imu->getAccelY();
    float az = _imu->getAccelZ();

    // Update the Madgwick filter
    _madgwickFilter.update(gx, gy, gz, ax, ay, az);

    // Retrieve attitude from the Madgwick filter
    state.attitude.roll = _madgwickFilter.getRoll();
    state.attitude.pitch = _madgwickFilter.getPitch();
    state.attitude.yaw = _madgwickFilter.getYaw();
}

// Performs sensor calibration by averaging a number of readings.
void AttitudeEstimator::calibrate()
{
    _imu->calibrate(_settings->calibration.mpuCalibrationReadings);
}
