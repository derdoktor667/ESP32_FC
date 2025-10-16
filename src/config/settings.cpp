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

static constexpr bool NVS_READ_WRITE_MODE = false;

void saveSettings()
{
    Serial.println("INFO: Saving settings to flash...");
    preferences.begin(PREFERENCES_NAMESPACE, NVS_READ_WRITE_MODE); // false = read-write mode

    // Mark settings as initialized
    preferences.putBool(INIT_KEY, true);

    // Save all settings from the struct
    preferences.putInt(NVSKeys::PID_ROLL_KP, settings.pidRoll.kp);
    preferences.putInt(NVSKeys::PID_ROLL_KI, settings.pidRoll.ki);
    preferences.putInt(NVSKeys::PID_ROLL_KD, settings.pidRoll.kd);
    preferences.putInt(NVSKeys::PID_PITCH_KP, settings.pidPitch.kp);
    preferences.putInt(NVSKeys::PID_PITCH_KI, settings.pidPitch.ki);
    preferences.putInt(NVSKeys::PID_PITCH_KD, settings.pidPitch.kd);
    preferences.putInt(NVSKeys::PID_YAW_KP, settings.pidYaw.kp);
    preferences.putInt(NVSKeys::PID_YAW_KI, settings.pidYaw.ki);
    preferences.putInt(NVSKeys::PID_YAW_KD, settings.pidYaw.kd);
    preferences.putFloat(NVSKeys::PID_INTEGRAL_LIMIT, settings.pidIntegralLimit);

    preferences.putFloat(NVSKeys::RATES_MAX_ANGLE_ROLL_PITCH, settings.rates.maxAngleRollPitch);
    preferences.putFloat(NVSKeys::RATES_MAX_RATE_YAW, settings.rates.maxRateYaw);
    preferences.putFloat(NVSKeys::RATES_MAX_RATE_ROLL_PITCH, settings.rates.maxRateRollPitch);

    preferences.putFloat(NVSKeys::FILTER_COMPLEMENTARY_TAU, settings.filter.complementaryFilterTau);
    preferences.putFloat(NVSKeys::FILTER_GYRO_LPF_CUTOFF_FREQ, settings.filter.gyroLpfCutoffFreq);
    preferences.putFloat(NVSKeys::FILTER_ACCEL_LPF_CUTOFF_FREQ, settings.filter.accelLpfCutoffFreq);
    preferences.putUChar(NVSKeys::FILTER_GYRO_LPF_STAGES, settings.filter.gyroLpfStages);
    preferences.putUChar(NVSKeys::FILTER_ACCEL_LPF_STAGES, settings.filter.accelLpfStages);
    preferences.putFloat(NVSKeys::FILTER_SAMPLE_FREQ, settings.filter.filterSampleFreq);

    // Receiver Settings
    preferences.putInt(NVSKeys::RX_PROTOCOL, (int)settings.receiverProtocol);
    preferences.putInt(NVSKeys::RX_MIN_VALUE, settings.receiver.ibusMinValue);
    preferences.putInt(NVSKeys::RX_MAX_VALUE, settings.receiver.ibusMaxValue);
    preferences.putInt(NVSKeys::RX_ARMING_THRESHOLD, settings.receiver.armingThreshold);
    preferences.putInt(NVSKeys::RX_FAILSAFE_THRESHOLD, settings.receiver.failsafeThreshold);

    // IMU Settings
    preferences.putInt(NVSKeys::IMU_PROTOCOL, (int)settings.imuProtocol);
    preferences.putInt(NVSKeys::IMU_LPF_BANDWIDTH, (int)settings.imuLpfBandwidth);

    // Save channel mapping
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
    {
        String key = String(NVSKeys::RX_CHANNEL_MAP_PREFIX) + String(i);
        preferences.putInt(key.c_str(), settings.channelMapping.channel[i]);
    }


    // Motor Settings
    preferences.putFloat(NVSKeys::MOTOR_IDLE_SPEED, settings.motorIdleSpeedPercent);
    preferences.putInt(NVSKeys::DSHOT_MODE_VAL, (int)settings.dshotMode);
    preferences.putBool(NVSKeys::ENFORCE_LOOP_TIME, settings.enforceLoopTime);

    Serial.println("INFO: Settings saved.");
}

void loadSettings()
{
    preferences.begin(PREFERENCES_NAMESPACE, NVS_READ_WRITE_MODE); // Open in read-write mode

    // Check if settings have been initialized before.
    bool initialized = preferences.getBool(INIT_KEY, false);

    if (initialized)
    {
        Serial.println("INFO: Saved settings found. Loading...");
        // Load all settings into the struct
        settings.pidRoll.kp = preferences.getInt(NVSKeys::PID_ROLL_KP, settings.pidRoll.kp);
        settings.pidRoll.ki = preferences.getInt(NVSKeys::PID_ROLL_KI, settings.pidRoll.ki);
        settings.pidRoll.kd = preferences.getInt(NVSKeys::PID_ROLL_KD, settings.pidRoll.kd);
        settings.pidPitch.kp = preferences.getInt(NVSKeys::PID_PITCH_KP, settings.pidPitch.kp);
        settings.pidPitch.ki = preferences.getInt(NVSKeys::PID_PITCH_KI, settings.pidPitch.ki);
        settings.pidPitch.kd = preferences.getInt(NVSKeys::PID_PITCH_KD, settings.pidPitch.kd);
        settings.pidYaw.kp = preferences.getInt(NVSKeys::PID_YAW_KP, settings.pidYaw.kp);
        settings.pidYaw.ki = preferences.getInt(NVSKeys::PID_YAW_KI, settings.pidYaw.ki);
        settings.pidYaw.kd = preferences.getInt(NVSKeys::PID_YAW_KD, settings.pidYaw.kd);
        settings.pidIntegralLimit = preferences.getFloat(NVSKeys::PID_INTEGRAL_LIMIT, settings.pidIntegralLimit);

        settings.rates.maxAngleRollPitch = preferences.getFloat(NVSKeys::RATES_MAX_ANGLE_ROLL_PITCH, settings.rates.maxAngleRollPitch);
        settings.rates.maxRateYaw = preferences.getFloat(NVSKeys::RATES_MAX_RATE_YAW, settings.rates.maxRateYaw);
        settings.rates.maxRateRollPitch = preferences.getFloat(NVSKeys::RATES_MAX_RATE_ROLL_PITCH, settings.rates.maxRateRollPitch);

        settings.filter.complementaryFilterTau = preferences.getFloat(NVSKeys::FILTER_COMPLEMENTARY_TAU, settings.filter.complementaryFilterTau);
        settings.filter.gyroLpfCutoffFreq = preferences.getFloat(NVSKeys::FILTER_GYRO_LPF_CUTOFF_FREQ, settings.filter.gyroLpfCutoffFreq);
        settings.filter.accelLpfCutoffFreq = preferences.getFloat(NVSKeys::FILTER_ACCEL_LPF_CUTOFF_FREQ, settings.filter.accelLpfCutoffFreq);
        settings.filter.gyroLpfStages = preferences.getUChar(NVSKeys::FILTER_GYRO_LPF_STAGES, settings.filter.gyroLpfStages);
        settings.filter.accelLpfStages = preferences.getUChar(NVSKeys::FILTER_ACCEL_LPF_STAGES, settings.filter.accelLpfStages);
        settings.filter.filterSampleFreq = preferences.getFloat(NVSKeys::FILTER_SAMPLE_FREQ, settings.filter.filterSampleFreq);

        // Receiver Settings
        settings.receiverProtocol = (ReceiverProtocol)preferences.getInt(NVSKeys::RX_PROTOCOL, (int)settings.receiverProtocol);
        settings.receiver.ibusMinValue = preferences.getInt(NVSKeys::RX_MIN_VALUE, settings.receiver.ibusMinValue);
        settings.receiver.ibusMaxValue = preferences.getInt(NVSKeys::RX_MAX_VALUE, settings.receiver.ibusMaxValue);
        settings.receiver.armingThreshold = preferences.getInt(NVSKeys::RX_ARMING_THRESHOLD, settings.receiver.armingThreshold);
        settings.receiver.failsafeThreshold = preferences.getInt(NVSKeys::RX_FAILSAFE_THRESHOLD, settings.receiver.failsafeThreshold);

        // IMU Settings
        settings.imuProtocol = (ImuProtocol)preferences.getInt(NVSKeys::IMU_PROTOCOL, (int)settings.imuProtocol);
        settings.imuLpfBandwidth = (LpfBandwidth)preferences.getInt(NVSKeys::IMU_LPF_BANDWIDTH, (int)settings.imuLpfBandwidth);


        // Motor Settings
        settings.motorIdleSpeedPercent = preferences.getFloat(NVSKeys::MOTOR_IDLE_SPEED, settings.motorIdleSpeedPercent);
        settings.dshotMode = (dshot_mode_t)preferences.getInt(NVSKeys::DSHOT_MODE_VAL, (int)settings.dshotMode);
        settings.enforceLoopTime = preferences.getBool(NVSKeys::ENFORCE_LOOP_TIME, settings.enforceLoopTime);

        // Load channel mapping
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
        {
            String key = String(NVSKeys::RX_CHANNEL_MAP_PREFIX) + String(i);
            settings.channelMapping.channel[i] = preferences.getInt(key.c_str(), settings.channelMapping.channel[i]);
        }
        Serial.println("INFO: Settings loaded.");
    }
    else
    {
        // This is the first run, or settings were cleared.
        // The 'settings' object already holds the default values from config.h.
        // We just need to save them to flash for the first time.
        Serial.println("INFO: No saved settings found. Saving default values...");
        saveSettings(); // This will also set the 'initialized' flag.
    }

    preferences.end();
}
