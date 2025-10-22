// settings.cpp
//
// This file implements the functions for managing persistent storage (Non-Volatile Storage - NVS)
// of the FlightControllerSettings. It handles saving and loading settings to/from the ESP32's flash memory.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include <Preferences.h>
#include "src/config/settings.h"

// Instantiate the global settings object, initialized with default values from config.h
FlightControllerSettings settings;

// Create a Preferences object to interact with non-volatile storage
Preferences preferences;

// Define a namespace for our settings to avoid conflicts
const char *PREFERENCES_NAMESPACE = "fc-settings";
// A key to check if settings have been initialized before
const char *INIT_KEY = "initialized";

static constexpr bool NVS_READ_WRITE_MODE = false; // false for read-write mode

static void _logSettingsStatus(const char *message)
{
    Serial.print("INFO: ");
    Serial.println(message);
}

void saveSettings()
{
    _logSettingsStatus("Saving settings to flash...");
    preferences.begin(PREFERENCES_NAMESPACE, NVS_READ_WRITE_MODE);

    // Mark settings as initialized
    preferences.putBool(INIT_KEY, true);

    // Save all settings from the struct
    preferences.putInt(NVSKeys::PID_ROLL_KP, settings.pid.roll.kp);
    preferences.putInt(NVSKeys::PID_ROLL_KI, settings.pid.roll.ki);
    preferences.putInt(NVSKeys::PID_ROLL_KD, settings.pid.roll.kd);
    preferences.putInt(NVSKeys::PID_PITCH_KP, settings.pid.pitch.kp);
    preferences.putInt(NVSKeys::PID_PITCH_KI, settings.pid.pitch.ki);
    preferences.putInt(NVSKeys::PID_PITCH_KD, settings.pid.pitch.kd);
    preferences.putInt(NVSKeys::PID_YAW_KP, settings.pid.yaw.kp);
    preferences.putInt(NVSKeys::PID_YAW_KI, settings.pid.yaw.ki);
    preferences.putInt(NVSKeys::PID_YAW_KD, settings.pid.yaw.kd);
    preferences.putFloat(NVSKeys::PID_INTEGRAL_LIMIT, settings.pid.integralLimit);

    preferences.putFloat(NVSKeys::RATES_MAX_ANGLE_ROLL_PITCH, settings.rates.maxAngleRollPitch);
    preferences.putFloat(NVSKeys::RATES_MAX_RATE_YAW, settings.rates.maxRateYaw);
    preferences.putFloat(NVSKeys::RATES_MAX_RATE_ROLL_PITCH, settings.rates.maxRateRollPitch);

    preferences.putFloat(NVSKeys::FILTER_COMPLEMENTARY_TAU, settings.filter.complementaryFilterTau);
    preferences.putFloat(NVSKeys::FILTER_GYRO_LPF_CUTOFF_FREQ, settings.filter.gyroLpfCutoffFreq);
    preferences.putFloat(NVSKeys::FILTER_ACCEL_LPF_CUTOFF_FREQ, settings.filter.accelLpfCutoffFreq);
    preferences.putUChar(NVSKeys::FILTER_GYRO_LPF_STAGES, settings.filter.gyroLpfStages);
    preferences.putUChar(NVSKeys::FILTER_ACCEL_LPF_STAGES, settings.filter.accelLpfStages);
    preferences.putFloat(NVSKeys::FILTER_SAMPLE_FREQ, settings.filter.filterSampleFreq);
    preferences.putBool(NVSKeys::FILTER_GYRO_NOTCH_ENABLE, settings.filter.enableGyroNotchFilter);
    preferences.putFloat(NVSKeys::FILTER_GYRO_NOTCH_FREQ, settings.filter.gyroNotchFreq);
    preferences.putFloat(NVSKeys::FILTER_GYRO_NOTCH_Q, settings.filter.gyroNotchQ);

    // Receiver Settings
    preferences.putInt(NVSKeys::RX_PROTOCOL, (int)settings.receiver.protocol);
    preferences.putInt(NVSKeys::RX_MIN_VALUE, settings.receiver.minValue);
    preferences.putInt(NVSKeys::RX_MAX_VALUE, settings.receiver.maxValue);
    preferences.putInt(NVSKeys::RX_ARMING_THRESHOLD, settings.receiver.armingThreshold);
    preferences.putInt(NVSKeys::RX_FAILSAFE_THRESHOLD, settings.receiver.failsafeThreshold);

    // IMU Settings
    preferences.putInt(NVSKeys::IMU_PROTOCOL, (int)settings.imu.protocol);
    preferences.putInt(NVSKeys::IMU_LPF_BANDWIDTH, (int)settings.imu.lpfBandwidth);
    preferences.putInt(NVSKeys::IMU_ROTATION, (int)settings.imu.rotation);

    // Save channel mapping
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
    {
        String key = String(NVSKeys::RX_CHANNEL_MAP_PREFIX) + String(i);
        preferences.putInt(key.c_str(), settings.receiver.channelMapping.channel[i]);
    }

    // Motor Settings
    preferences.putFloat(NVSKeys::MOTOR_IDLE_SPEED, settings.motor.idleSpeedPercent);
    preferences.putInt(NVSKeys::DSHOT_MODE_VAL, (int)settings.motor.dshotMode);
    preferences.putBool(NVSKeys::ENFORCE_LOOP_TIME, settings.logging.enforceLoopTime);

    // Calibration Settings
    preferences.putInt(NVSKeys::CALIBRATION_MPU_READINGS, settings.calibration.mpuCalibrationReadings);
    preferences.putFloat(NVSKeys::CALIBRATION_ACCEL_Z_GRAVITY, settings.calibration.accelZGravity);

    // Logging Settings
    preferences.putULong(NVSKeys::LOGGING_PRINT_INTERVAL_MS, settings.logging.printIntervalMs);
    preferences.putBool(NVSKeys::LOGGING_ENABLE, settings.logging.enableLogging);
    preferences.putBool(NVSKeys::BENCH_RUN_MODE_ENABLE, settings.logging.enableBenchRunMode);
    preferences.putString(NVSKeys::LOGGING_TEST_STRING, settings.logging.testString);
    preferences.putUShort(NVSKeys::FIRMWARE_BUILD_ID, FIRMWARE_BUILD_ID);

    _logSettingsStatus("Settings saved.");
}

void loadSettings()
{
    preferences.begin(PREFERENCES_NAMESPACE, NVS_READ_WRITE_MODE);

    // Check if settings have been initialized before.
    static constexpr uint16_t DEFAULT_FIRMWARE_BUILD_ID = 0;
    bool initialized = preferences.getBool(INIT_KEY, false);
    uint16_t storedBuildId = preferences.getUShort(NVSKeys::FIRMWARE_BUILD_ID, DEFAULT_FIRMWARE_BUILD_ID); // Default to 0 if not found

    // If not initialized, or firmware build ID mismatch, reset to defaults.
    if (!initialized || storedBuildId != FIRMWARE_BUILD_ID)
    {
        _logSettingsStatus("No saved settings found or firmware updated. Saving default values...");
        settings = FlightControllerSettings(); // Reset to default values
        saveSettings(); // This will also set the 'initialized' flag and save the new build ID.
    }
    else
    {
        _logSettingsStatus("Saved settings found. Loading...");
        // Load all settings into the struct
        settings.pid.roll.kp = preferences.getInt(NVSKeys::PID_ROLL_KP, settings.pid.roll.kp);
        settings.pid.roll.ki = preferences.getInt(NVSKeys::PID_ROLL_KI, settings.pid.roll.ki);
        settings.pid.roll.kd = preferences.getInt(NVSKeys::PID_ROLL_KD, settings.pid.roll.kd);
        settings.pid.pitch.kp = preferences.getInt(NVSKeys::PID_PITCH_KP, settings.pid.pitch.kp);
        settings.pid.pitch.ki = preferences.getInt(NVSKeys::PID_PITCH_KI, settings.pid.pitch.ki);
        settings.pid.pitch.kd = preferences.getInt(NVSKeys::PID_PITCH_KD, settings.pid.pitch.kd);
        settings.pid.yaw.kp = preferences.getInt(NVSKeys::PID_YAW_KP, settings.pid.yaw.kp);
        settings.pid.yaw.ki = preferences.getInt(NVSKeys::PID_YAW_KI, settings.pid.yaw.ki);
        settings.pid.yaw.kd = preferences.getInt(NVSKeys::PID_YAW_KD, settings.pid.yaw.kd);
        settings.pid.integralLimit = preferences.getFloat(NVSKeys::PID_INTEGRAL_LIMIT, settings.pid.integralLimit);

        settings.rates.maxAngleRollPitch = preferences.getFloat(NVSKeys::RATES_MAX_ANGLE_ROLL_PITCH, settings.rates.maxAngleRollPitch);
        settings.rates.maxRateYaw = preferences.getFloat(NVSKeys::RATES_MAX_RATE_YAW, settings.rates.maxRateYaw);
        settings.rates.maxRateRollPitch = preferences.getFloat(NVSKeys::RATES_MAX_RATE_ROLL_PITCH, settings.rates.maxRateRollPitch);

        settings.filter.complementaryFilterTau = preferences.getFloat(NVSKeys::FILTER_COMPLEMENTARY_TAU, settings.filter.complementaryFilterTau);
        settings.filter.gyroLpfCutoffFreq = preferences.getFloat(NVSKeys::FILTER_GYRO_LPF_CUTOFF_FREQ, settings.filter.gyroLpfCutoffFreq);
        settings.filter.accelLpfCutoffFreq = preferences.getFloat(NVSKeys::FILTER_ACCEL_LPF_CUTOFF_FREQ, settings.filter.accelLpfCutoffFreq);
        settings.filter.gyroLpfStages = preferences.getUChar(NVSKeys::FILTER_GYRO_LPF_STAGES, settings.filter.gyroLpfStages);
        settings.filter.accelLpfStages = preferences.getUChar(NVSKeys::FILTER_ACCEL_LPF_STAGES, settings.filter.accelLpfStages);
        settings.filter.filterSampleFreq = preferences.getFloat(NVSKeys::FILTER_SAMPLE_FREQ, settings.filter.filterSampleFreq);
        settings.filter.enableGyroNotchFilter = preferences.getBool(NVSKeys::FILTER_GYRO_NOTCH_ENABLE, settings.filter.enableGyroNotchFilter);
        settings.filter.gyroNotchFreq = preferences.getFloat(NVSKeys::FILTER_GYRO_NOTCH_FREQ, settings.filter.gyroNotchFreq);
        settings.filter.gyroNotchQ = preferences.getFloat(NVSKeys::FILTER_GYRO_NOTCH_Q, settings.filter.gyroNotchQ);

        // Receiver Settings
        settings.receiver.protocol = (ReceiverProtocol)preferences.getInt(NVSKeys::RX_PROTOCOL, (int)settings.receiver.protocol);
        settings.receiver.minValue = preferences.getInt(NVSKeys::RX_MIN_VALUE, settings.receiver.minValue);
        settings.receiver.maxValue = preferences.getInt(NVSKeys::RX_MAX_VALUE, settings.receiver.maxValue);
        settings.receiver.armingThreshold = preferences.getInt(NVSKeys::RX_ARMING_THRESHOLD, settings.receiver.armingThreshold);
        settings.receiver.failsafeThreshold = preferences.getInt(NVSKeys::RX_FAILSAFE_THRESHOLD, settings.receiver.failsafeThreshold);

        // IMU Settings
        settings.imu.protocol = (ImuProtocol)preferences.getInt(NVSKeys::IMU_PROTOCOL, (int)settings.imu.protocol);
        settings.imu.lpfBandwidth = (LpfBandwidth)preferences.getInt(NVSKeys::IMU_LPF_BANDWIDTH, (int)settings.imu.lpfBandwidth);
        settings.imu.rotation = (ImuRotation)preferences.getInt(NVSKeys::IMU_ROTATION, (int)settings.imu.rotation);

        // Motor Settings
        settings.motor.idleSpeedPercent = preferences.getFloat(NVSKeys::MOTOR_IDLE_SPEED, settings.motor.idleSpeedPercent);
        settings.motor.dshotMode = (dshot_mode_t)preferences.getInt(NVSKeys::DSHOT_MODE_VAL, (int)settings.motor.dshotMode);
        settings.logging.enforceLoopTime = preferences.getBool(NVSKeys::ENFORCE_LOOP_TIME, settings.logging.enforceLoopTime);

        // Calibration Settings
        settings.calibration.mpuCalibrationReadings = preferences.getInt(NVSKeys::CALIBRATION_MPU_READINGS, settings.calibration.mpuCalibrationReadings);
        settings.calibration.accelZGravity = preferences.getFloat(NVSKeys::CALIBRATION_ACCEL_Z_GRAVITY, settings.calibration.accelZGravity);

        // Logging Settings
        settings.logging.printIntervalMs = preferences.getULong(NVSKeys::LOGGING_PRINT_INTERVAL_MS, settings.logging.printIntervalMs);
        settings.logging.enableLogging = preferences.getBool(NVSKeys::LOGGING_ENABLE, settings.logging.enableLogging);
        settings.logging.enableBenchRunMode = preferences.getBool(NVSKeys::BENCH_RUN_MODE_ENABLE, settings.logging.enableBenchRunMode);
        settings.logging.testString = preferences.getString(NVSKeys::LOGGING_TEST_STRING, settings.logging.testString);

        // Load channel mapping
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
        {
            String key = String(NVSKeys::RX_CHANNEL_MAP_PREFIX) + String(i);
            settings.receiver.channelMapping.channel[i] = preferences.getInt(key.c_str(), settings.receiver.channelMapping.channel[i]);
        }
        _logSettingsStatus("Settings loaded.");
    }

    preferences.end();
}
