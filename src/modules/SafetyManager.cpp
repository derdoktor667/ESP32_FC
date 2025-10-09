#include "SafetyManager.h"
#include "../config.h"
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
    // Failsafe is activated if the receiver reports signal loss OR if the dedicated failsafe switch is active.
    if (_receiver.hasFailsafe() || (failsafeSwitchChannelValue > _settings.receiver.failsafeThreshold))
    {
        // If failsafe was not previously active, activate it and force disarm.
        if (!state.isFailsafeActive)
        {
            state.isFailsafeActive = true;
            state.isArmed = false; // Crucial: Always disarm when failsafe is active.
            Serial.println("FAILSAFE ACTIVATED - MOTORS STOPPED");
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
            Serial.println("Failsafe deactivated");
        }
    }

    // --- Arming/Disarming Logic (Only processed if failsafe is NOT active) ---
    // Check if the arming switch is in the "armed" position and the drone is not yet armed.
    if (armingChannelValue > _settings.receiver.armingThreshold && !state.isArmed)
    {
        state.isArmed = true;
        Serial.println("ARMED");
    }
    // Check if the arming switch is in the "disarmed" position and the drone is currently armed.
    else if (armingChannelValue < _settings.receiver.armingThreshold && state.isArmed)
    {
        state.isArmed = false;
        Serial.println("DISARMED");
    }
}
