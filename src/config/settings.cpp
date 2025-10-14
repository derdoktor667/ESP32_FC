#include <Preferences.h>
#include "src/config/settings.h"

// Instantiate the global settings object
// This will be initialized with the default values from the struct definition in config.h
FlightControllerSettings settings;

// Create a Preferences object to interact with non-volatile storage
Preferences preferences;

// Define a namespace for our settings to avoid conflicts
const char *PREFERENCES_NAMESPACE = "fc-settings";
// A key to check if settings have been initialized before
const char *INIT_KEY = "initialized";

void saveSettings()
{
    Serial.println("INFO: Saving settings to flash...");
    preferences.begin(PREFERENCES_NAMESPACE, false); // false = read-write mode

    // Mark settings as initialized
    preferences.putBool(INIT_KEY, true);

    // Save all settings from the struct
    preferences.putInt("pid.r.kp", settings.pidRoll.kp);
    preferences.putInt("pid.r.ki", settings.pidRoll.ki);
    preferences.putInt("pid.r.kd", settings.pidRoll.kd);
    preferences.putInt("pid.p.kp", settings.pidPitch.kp);
    preferences.putInt("pid.p.ki", settings.pidPitch.ki);
    preferences.putInt("pid.p.kd", settings.pidPitch.kd);
    preferences.putInt("pid.y.kp", settings.pidYaw.kp);
    preferences.putInt("pid.y.ki", settings.pidYaw.ki);
    preferences.putInt("pid.y.kd", settings.pidYaw.kd);
    preferences.putFloat("pid.lim", settings.pidIntegralLimit);

    preferences.putFloat("rate.angle", settings.rates.maxAngleRollPitch);
    preferences.putFloat("rate.y", settings.rates.maxRateYaw);
    preferences.putFloat("rate.acro_rp", settings.rates.maxRateRollPitch);

    preferences.putFloat("filter.madgwick.sample_freq", settings.filter.madgwickSampleFreq);
    preferences.putFloat("filter.madgwick.beta", settings.filter.madgwickBeta);

    // Receiver Settings
    preferences.putInt("rx.proto", (int)settings.receiverProtocol);
    preferences.putInt("rx.min", settings.receiver.ibusMinValue);
    preferences.putInt("rx.max", settings.receiver.ibusMaxValue);
    preferences.putInt("rx.arming", settings.receiver.armingThreshold);
    preferences.putInt("rx.failsafe", settings.receiver.failsafeThreshold);

    // IMU Settings
    preferences.putInt("imu.proto", (int)settings.imuProtocol);

    // Save channel mapping
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
    {
        String key = "rx.map." + String(i);
        preferences.putInt(key.c_str(), settings.channelMapping.channel[i]);
    }


    // Motor Settings
    preferences.putFloat("motor.idle", settings.motorIdleSpeedPercent);
    preferences.putInt("dshot_mode_val", (int)settings.dshotMode);

    Serial.println("INFO: Settings saved.");
}

void loadSettings()
{
    preferences.begin(PREFERENCES_NAMESPACE, false); // Open in read-write mode

    // Check if settings have been initialized before.
    bool initialized = preferences.getBool(INIT_KEY, false);

    if (initialized)
    {
        Serial.println("INFO: Saved settings found. Loading...");
        // Load all settings into the struct
        settings.pidRoll.kp = preferences.getInt("pid.r.kp", settings.pidRoll.kp);
        settings.pidRoll.ki = preferences.getInt("pid.r.ki", settings.pidRoll.ki);
        settings.pidRoll.kd = preferences.getInt("pid.r.kd", settings.pidRoll.kd);
        settings.pidPitch.kp = preferences.getInt("pid.p.kp", settings.pidPitch.kp);
        settings.pidPitch.ki = preferences.getInt("pid.p.ki", settings.pidPitch.ki);
        settings.pidPitch.kd = preferences.getInt("pid.p.kd", settings.pidPitch.kd);
        settings.pidYaw.kp = preferences.getInt("pid.y.kp", settings.pidYaw.kp);
        settings.pidYaw.ki = preferences.getInt("pid.y.ki", settings.pidYaw.ki);
        settings.pidYaw.kd = preferences.getInt("pid.y.kd", settings.pidYaw.kd);
        settings.pidIntegralLimit = preferences.getFloat("pid.lim", settings.pidIntegralLimit);

        settings.rates.maxAngleRollPitch = preferences.getFloat("rate.angle", settings.rates.maxAngleRollPitch);
        settings.rates.maxRateYaw = preferences.getFloat("rate.y", settings.rates.maxRateYaw);
        settings.rates.maxRateRollPitch = preferences.getFloat("rate.acro_rp", settings.rates.maxRateRollPitch);

        settings.filter.madgwickSampleFreq = preferences.getFloat("filter.madgwick.sample_freq", settings.filter.madgwickSampleFreq);
        settings.filter.madgwickBeta = preferences.getFloat("filter.madgwick.beta", settings.filter.madgwickBeta);

        // Receiver Settings
        settings.receiverProtocol = (ReceiverProtocol)preferences.getInt("rx.proto", (int)settings.receiverProtocol);
        settings.receiver.ibusMinValue = preferences.getInt("rx.min", settings.receiver.ibusMinValue);
        settings.receiver.ibusMaxValue = preferences.getInt("rx.max", settings.receiver.ibusMaxValue);
        settings.receiver.armingThreshold = preferences.getInt("rx.arming", settings.receiver.armingThreshold);
        settings.receiver.failsafeThreshold = preferences.getInt("rx.failsafe", settings.receiver.failsafeThreshold);

        // IMU Settings
        settings.imuProtocol = (ImuProtocol)preferences.getInt("imu.proto", (int)settings.imuProtocol);


        // Motor Settings
        settings.motorIdleSpeedPercent = preferences.getFloat("motor.idle", settings.motorIdleSpeedPercent);
        settings.dshotMode = (dshot_mode_t)preferences.getInt("dshot_mode_val", (int)settings.dshotMode);

        // Load channel mapping
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
        {
            String key = "rx.map." + String(i);
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
