// SafetyManager.cpp
//
// This file implements the SafetyManager class, which is responsible for
// monitoring receiver input and managing the arming, disarming, and failsafe
// states of the flight controller. It ensures the drone operates safely based
// on predefined thresholds and receiver status.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "src/modules/SafetyManager.h"
#include <Arduino.h>

// Constructor: Initializes the SafetyManager with references to the receiver and settings.
SafetyManager::SafetyManager(ReceiverInterface &receiver, const FlightControllerSettings &settings)
    : _receiver(receiver), _settings(settings)
{
}

// Performs the safety checks for failsafe and arming/disarming.
// This method updates the `isArmed` and `isFailsafeActive` flags in the FlightState.
void SafetyManager::update(FlightState &state)
{
    // Retrieve the current values of the arming and failsafe switches from the receiver channels.
    uint16_t armingChannelValue = state.receiverChannels[_settings.channelMapping.channel[ARM_SWITCH]];
    uint16_t failsafeSwitchChannelValue = state.receiverChannels[_settings.channelMapping.channel[FAILSAFE_SWITCH]];

    // --- Failsafe Logic (Highest Priority) ---
    bool receiverFailsafeActive = _receiver.hasFailsafe();
    bool failsafeSwitchEngaged = (failsafeSwitchChannelValue > _settings.receiver.failsafeThreshold);

    // Failsafe is activated if the receiver reports signal loss OR if the dedicated failsafe switch is active.
    if (receiverFailsafeActive || failsafeSwitchEngaged)
    {
        // If failsafe was not previously active, activate it and force disarm.
        if (!state.isFailsafeActive)
        {
            state.isFailsafeActive = true;
            state.isArmed = false; // Crucial: Always disarm when failsafe is active.
            _logSafetyStatus("FAILSAFE ACTIVATED - MOTORS STOPPED");
        }
        // When in failsafe, no further arming/disarming logic should be processed.
        // Motors are commanded to stop by MotorMixer.
        return;
    }
    else
    {
        // If failsafe was active but conditions are now clear, deactivate it.
        if (state.isFailsafeActive)
        {
            state.isFailsafeActive = false;
            _logSafetyStatus("Failsafe deactivated.");
        }
    }

    // --- Arming/Disarming Logic (Only processed if failsafe is NOT active) ---
    bool armingSwitchEngaged = (armingChannelValue > _settings.receiver.armingThreshold);
    bool disarmingSwitchEngaged = (armingChannelValue < _settings.receiver.armingThreshold);

    // Check if the arming switch is in the "armed" position and the drone is not yet armed.
    if (armingSwitchEngaged && !state.isArmed)
    {
        state.isArmed = true;
        _logSafetyStatus("ARMED");
    }
    // Check if the arming switch is in the "disarmed" position and the drone is currently armed.
    else if (disarmingSwitchEngaged && state.isArmed)
    {
        state.isArmed = false;
        _logSafetyStatus("DISARMED");
    }
}

void SafetyManager::_logSafetyStatus(const char *message)
{
    Serial.print("INFO: ");
    Serial.println(message);
}
