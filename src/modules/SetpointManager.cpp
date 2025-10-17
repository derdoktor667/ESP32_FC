// SetpointManager.cpp
//
// This file implements the SetpointManager class, which is responsible for
// translating raw receiver inputs into desired flight setpoints (roll, pitch, yaw)
// based on the current flight mode. It scales and maps stick movements to
// appropriate control targets for the PID controllers.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "src/modules/SetpointManager.h"

#include <Arduino.h>

// Constructor: Initializes the SetpointManager with references to the receiver and settings.
SetpointManager::SetpointManager(ReceiverInterface &receiver, const FlightControllerSettings &settings)
    : _receiver(receiver), _settings(settings)
{
}

// Calculates the target setpoints for roll, pitch, and yaw based on receiver input and flight mode.
// The calculated setpoints are stored in the provided FlightState object.
void SetpointManager::update(FlightState &state)
{
    // Receiver channel values are assumed to be already updated in FlightController::runLoop().
    // Use the values directly from FlightState.

    // Determine the current flight mode based on the mapped flight mode switch channel.
    uint16_t flightModeChannelValue = state.receiverChannels[_settings.channelMapping.channel[FLIGHT_MODE_SWITCH] - RECEIVER_CHANNEL_OFFSET];
    if (flightModeChannelValue < FLIGHT_MODE_ACRO_THRESHOLD) // Example threshold for ACRO_MODE
    {
        state.currentFlightMode = ACRO_MODE;
    }
    else if (flightModeChannelValue > FLIGHT_MODE_ANGLE_THRESHOLD) // Example threshold for ANGLE_MODE
    {
        state.currentFlightMode = ANGLE_MODE;
    }
    // Intermediate values could imply other modes or be ignored.

    // Calculate setpoints for Roll, Pitch, Yaw, and Throttle based on flight mode and mapped channels.
    uint16_t rollChannelValue = state.receiverChannels[_settings.channelMapping.channel[ROLL] - RECEIVER_CHANNEL_OFFSET];
    uint16_t pitchChannelValue = state.receiverChannels[_settings.channelMapping.channel[PITCH] - RECEIVER_CHANNEL_OFFSET];
    uint16_t yawChannelValue = state.receiverChannels[_settings.channelMapping.channel[YAW] - RECEIVER_CHANNEL_OFFSET];
    uint16_t throttleChannelValue = state.receiverChannels[_settings.channelMapping.channel[THROTTLE] - RECEIVER_CHANNEL_OFFSET];

    // Map receiver input to target setpoints. The `map` function is an Arduino utility function.
    if (state.currentFlightMode == ANGLE_MODE)
    {
        // In ANGLE_MODE, stick input directly maps to a target angle (degrees).
        state.setpoints.roll = _calculateSetpoint(rollChannelValue, _settings.rates.maxAngleRollPitch);
        state.setpoints.pitch = _calculateSetpoint(pitchChannelValue, _settings.rates.maxAngleRollPitch);
    }
    else // ACRO_MODE (or any other rate-based mode)
    {
        // In ACRO_MODE, stick input maps to a target rotation rate (degrees/second).
        state.setpoints.roll = _calculateSetpoint(rollChannelValue, _settings.rates.maxRateRollPitch);
        state.setpoints.pitch = _calculateSetpoint(pitchChannelValue, _settings.rates.maxRateRollPitch);
    }

    // Yaw control is typically always rate-based, regardless of flight mode.
    state.setpoints.yaw = _calculateSetpoint(yawChannelValue, _settings.rates.maxRateYaw);

    // Map throttle input from receiver's raw range to the DShot throttle range.
    state.throttle = _calculateThrottle(throttleChannelValue);
}

float SetpointManager::_calculateSetpoint(uint16_t channelValue, float maxRateOrAngle) const
{
    return (float)(channelValue - _settings.receiver.ibusMinValue) * (SETPOINT_SCALING_FACTOR * maxRateOrAngle) / (_settings.receiver.ibusMaxValue - _settings.receiver.ibusMinValue) - maxRateOrAngle;
}

float SetpointManager::_calculateThrottle(uint16_t throttleChannelValue) const
{
    return (float)(throttleChannelValue - _settings.receiver.ibusMinValue) * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) / (_settings.receiver.ibusMaxValue - _settings.receiver.ibusMinValue) + DSHOT_MIN_THROTTLE;
}
