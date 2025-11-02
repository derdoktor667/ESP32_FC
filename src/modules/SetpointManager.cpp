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
#include "src/config/config.h" // Required for FLIGHT_MODE_ACRO_THRESHOLD, FLIGHT_MODE_ANGLE_THRESHOLD, RECEIVER_CHANNEL_OFFSET, ROLL, PITCH, YAW, THROTTLE, SETPOINT_SCALING_FACTOR, DSHOT_MAX_THROTTLE, DSHOT_MIN_THROTTLE
#include <Arduino.h>

// Constructor: Initializes the SetpointManager with references to the receiver and settings.
SetpointManager::SetpointManager(ReceiverInterface &receiver, const ReceiverSettings &receiverSettings, const FlightRateSettings &rateSettings)
    : _receiver(receiver), _receiverSettings(receiverSettings), _rateSettings(rateSettings)
{
}

// Calculates the target setpoints for roll, pitch, and yaw based on receiver input and flight mode.
// The calculated setpoints are stored in the provided FlightState object.
void SetpointManager::update(FlightState &state)
{
    // Receiver channel values are assumed to be already updated in FlightController::runLoop().
    // Use the values directly from FlightState.

    // Determine the current flight mode based on the mapped flight mode switch channel.
    uint16_t flightModeChannelValue = state.receiverChannels[_receiverSettings.channelMapping.channel[FLIGHT_MODE_SWITCH]];
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
    uint16_t rollChannelValue = state.receiverChannels[_receiverSettings.channelMapping.channel[ROLL]];
    uint16_t pitchChannelValue = state.receiverChannels[_receiverSettings.channelMapping.channel[PITCH]];
    uint16_t yawChannelValue = state.receiverChannels[_receiverSettings.channelMapping.channel[YAW]];
    uint16_t throttleChannelValue = state.receiverChannels[_receiverSettings.channelMapping.channel[THROTTLE]];

    // Map receiver input to target setpoints. The `map` function is an Arduino utility function.
    if (state.currentFlightMode == ANGLE_MODE)
    {
        // In ANGLE_MODE, stick input directly maps to a target angle (degrees).
        state.setpoints.roll = _calculateSetpoint(rollChannelValue, _rateSettings.maxAngleRollPitch);
        state.setpoints.pitch = _calculateSetpoint(pitchChannelValue, _rateSettings.maxAngleRollPitch);
    }
    else // ACRO_MODE (or any other rate-based mode)
    {
        // In ACRO_MODE, stick input maps to a target rotation rate (degrees/second).
        state.setpoints.roll = _calculateSetpoint(rollChannelValue, _rateSettings.maxRateRollPitch);
        state.setpoints.pitch = _calculateSetpoint(pitchChannelValue, _rateSettings.maxRateRollPitch);
    }

    // Yaw control is typically always rate-based, regardless of flight mode.
    state.setpoints.yaw = _calculateSetpoint(yawChannelValue, _rateSettings.maxRateYaw);

    // Map throttle input from receiver's raw range to the DShot throttle range.
    state.throttle = _calculateThrottle(throttleChannelValue);
}

float SetpointManager::_calculateSetpoint(uint16_t channelValue, float maxRateOrAngle) const
{
    return (float)(channelValue - _receiverSettings.minValue) * (SETPOINT_SCALING_FACTOR * maxRateOrAngle) / (_receiverSettings.maxValue - _receiverSettings.minValue) - maxRateOrAngle;
}

float SetpointManager::_calculateThrottle(uint16_t throttleChannelValue) const
{
    return (float)(throttleChannelValue - _receiverSettings.minValue) * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE) / (_receiverSettings.maxValue - _receiverSettings.minValue) + DSHOT_MIN_THROTTLE;
}
