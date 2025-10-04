#include <Preferences.h>
#include "settings.h"

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
    preferences.begin(PREFERENCES_NAMESPACE, false); // false = read-write mode

    // Save all settings from the struct
    preferences.putFloat("pid.r.kp", settings.pidRoll.kp);
    preferences.putFloat("pid.r.ki", settings.pidRoll.ki);
    preferences.putFloat("pid.r.kd", settings.pidRoll.kd);
    preferences.putFloat("pid.p.kp", settings.pidPitch.kp);
    preferences.putFloat("pid.p.ki", settings.pidPitch.ki);
    preferences.putFloat("pid.p.kd", settings.pidPitch.kd);
    preferences.putFloat("pid.y.kp", settings.pidYaw.kp);
    preferences.putFloat("pid.y.ki", settings.pidYaw.ki);
    preferences.putFloat("pid.y.kd", settings.pidYaw.kd);
    preferences.putFloat("pid.lim", settings.pidIntegralLimit);

    preferences.putFloat("rate.r_p", settings.rates.targetAngleRollPitch);
    preferences.putFloat("rate.y", settings.rates.targetRateYaw);
    preferences.putFloat("rate.acro", settings.rates.targetRateRollPitch);

    preferences.putFloat("filter.gain", settings.filter.complementaryFilterGain);

    // Mark that settings have been initialized
    preferences.putBool(INIT_KEY, true);

    preferences.end();
    Serial.println("Settings saved to flash memory.");
}

void loadSettings()
{
    preferences.begin(PREFERENCES_NAMESPACE, true); // true = read-only mode

    // Check if the settings have ever been saved
    bool initialized = preferences.getBool(INIT_KEY, false);

    if (initialized)
    {
        Serial.println("Loading settings from flash memory...");
        // Load all settings into the struct
        settings.pidRoll.kp = preferences.getFloat("pid.r.kp", settings.pidRoll.kp);
        settings.pidRoll.ki = preferences.getFloat("pid.r.ki", settings.pidRoll.ki);
        settings.pidRoll.kd = preferences.getFloat("pid.r.kd", settings.pidRoll.kd);
        settings.pidPitch.kp = preferences.getFloat("pid.p.kp", settings.pidPitch.kp);
        settings.pidPitch.ki = preferences.getFloat("pid.p.ki", settings.pidPitch.ki);
        settings.pidPitch.kd = preferences.getFloat("pid.p.kd", settings.pidPitch.kd);
        settings.pidYaw.kp = preferences.getFloat("pid.y.kp", settings.pidYaw.kp);
        settings.pidYaw.ki = preferences.getFloat("pid.y.ki", settings.pidYaw.ki);
        settings.pidYaw.kd = preferences.getFloat("pid.y.kd", settings.pidYaw.kd);
        settings.pidIntegralLimit = preferences.getFloat("pid.lim", settings.pidIntegralLimit);

        settings.rates.targetAngleRollPitch = preferences.getFloat("rate.r_p", settings.rates.targetAngleRollPitch);
        settings.rates.targetRateYaw = preferences.getFloat("rate.y", settings.rates.targetRateYaw);
        settings.rates.targetRateRollPitch = preferences.getFloat("rate.acro", settings.rates.targetRateRollPitch);

        settings.filter.complementaryFilterGain = preferences.getFloat("filter.gain", settings.filter.complementaryFilterGain);
    }
    else
    {
        Serial.println("No saved settings found. Initializing with default values.");
        // If no settings are found, save the current default values for the first time.
        // Need to switch to read-write mode for this.
        preferences.end(); // End read-only session
        saveSettings();    // This will save the defaults and print the confirmation
        return;            // Exit to avoid closing preferences twice
    }

    preferences.end();
}
