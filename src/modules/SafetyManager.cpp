#include "SafetyManager.h"
#include "../config.h"
#include <Arduino.h>

// Constructor: Initializes with references to the receiver and settings.
SafetyManager::SafetyManager(ReceiverInterface &receiver, const FlightControllerSettings &settings)
    : _receiver(receiver), _settings(settings)
{
}

// Performs the safety checks for failsafe and arming.
void SafetyManager::update(FlightState &state)
{
    // Read channel values for arming and failsafe switches
    uint16_t armingChannelValue = state.receiverChannels[_settings.channelMapping.channel[ARM_SWITCH]];
    uint16_t failsafeSwitchChannelValue = state.receiverChannels[_settings.channelMapping.channel[FAILSAFE_SWITCH]];

    // Failsafe has the highest priority.
    // Trigger failsafe if receiver signal is lost OR if the failsafe switch is active.
    if (_receiver.hasFailsafe() || (failsafeSwitchChannelValue > _settings.receiver.failsafeThreshold))
    {
        if (!state.isFailsafeActive)
        {
            state.isFailsafeActive = true;
            state.isArmed = false; // Force disarm on failsafe
            Serial.println("FAILSAFE ACTIVATED - MOTORS STOPPED");
        }
        // When in failsafe, we immediately exit to ensure motors are not commanded.
        return;
    }
    else
    {
        if (state.isFailsafeActive)
        {
            state.isFailsafeActive = false;
            Serial.println("Failsafe deactivated");
        }
    }

    // Arming logic is only processed if failsafe is not active.
    if (armingChannelValue > _settings.receiver.armingThreshold && !state.isArmed)
    {
        state.isArmed = true;
        Serial.println("ARMED");
    }
    else if (armingChannelValue < _settings.receiver.armingThreshold && state.isArmed)
    {
        state.isArmed = false;
        Serial.println("DISARMED");
    }
}
