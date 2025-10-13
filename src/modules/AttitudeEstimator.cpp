#include "src/modules/AttitudeEstimator.h"
#include "src/config/config.h"
#include <Arduino.h>

// Default constructor: Initializes pointers to nullptr and Madgwick filter with dummy values.
// Actual initialization happens in init() once settings are available.
AttitudeEstimator::AttitudeEstimator()
    : _imu(nullptr), _settings(nullptr),
      _madgwickFilter(0.0f, 0.0f)
{
}

// Init method: Sets up references to IMU and settings, then properly initializes the Madgwick filter.
void AttitudeEstimator::init(ImuInterface &imu, const FlightControllerSettings &settings)
{
    _imu = &imu;
    _settings = &settings;
    // Re-initialize Madgwick filter with actual settings from config.h
    _madgwickFilter = MadgwickFilter(_settings->filter.madgwickSampleFreq, _settings->filter.madgwickBeta);
}

// Initializes the IMU sensor and performs the initial calibration.
void AttitudeEstimator::begin()
{
    Serial.println("Initializing IMU...");
    // Attempt to initialize the IMU. If it fails, halt the program as it's a critical component.
    if (!_imu->begin())
    {
        Serial.println("Failed to initialize IMU. Halting.");
        while (1)
        {
            delay(IMU_INIT_FAIL_DELAY_MS); // Keep delaying to indicate a halt
        }
    }
    Serial.println("IMU initialized.");

    // Perform initial calibration of the IMU to remove biases.
    calibrate();
}

// Performs one cycle of attitude calculation: reads IMU, updates filter, and stores attitude in state.
void AttitudeEstimator::update(FlightState &state)
{
    _imu->update(); // Read the latest raw sensor data from the IMU

    // Get calibrated sensor data from the IMU interface.
    // Gyroscope data should be in rad/s for the Madgwick filter.
    // Accelerometer data should be in g's.
    float gx = radians(_imu->getGyroRollRate());
    float gy = radians(_imu->getGyroPitchRate());
    float gz = radians(_imu->getGyroYawRate());
    float ax = _imu->getAccelX();
    float ay = _imu->getAccelY();
    float az = _imu->getAccelZ();

    // Update the Madgwick filter with the latest sensor readings.
    // This performs sensor fusion to estimate orientation.
    _madgwickFilter.update(gx, gy, gz, ax, ay, az);

    // Retrieve the calculated attitude (roll, pitch, yaw) from the Madgwick filter
    // and store it in the FlightState.
    state.attitude.roll = _madgwickFilter.getRoll();
    state.attitude.pitch = _madgwickFilter.getPitch();
    state.attitude.yaw = _madgwickFilter.getYaw();
}

// Performs sensor calibration by averaging a number of readings.
// This method delegates the calibration process to the IMU interface.
void AttitudeEstimator::calibrate()
{
    _imu->calibrate(_settings->calibration.mpuCalibrationReadings);
}
