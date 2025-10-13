#include "src/modules/SetpointManager.h"

#include <Arduino.h>

// Constructor: Initializes the SetpointManager with references to the receiver and settings.
SetpointManager::SetpointManager(ReceiverInterface &receiver, const FlightControllerSettings &settings)
    : _receiver(receiver), _settings(settings)
{
}

// Calculates the target setpoints for roll, pitch, and yaw based on receiver input and flight mode.
void SetpointManager::update(FlightState &state)
{
    // 1. Receiver channel values are assumed to be already updated in FlightController::runLoop().
    //    Use the values directly from FlightState.

    // 2. Determine the current flight mode based on the mapped flight mode switch channel.
    uint16_t flightModeChannelValue = state.receiverChannels[_settings.channelMapping.channel[FLIGHT_MODE_SWITCH]];
    if (flightModeChannelValue < FLIGHT_MODE_ACRO_THRESHOLD) // Example threshold for ACRO_MODE
    {
        state.currentFlightMode = ACRO_MODE;
    }
    else if (flightModeChannelValue > FLIGHT_MODE_ANGLE_THRESHOLD) // Example threshold for ANGLE_MODE
    {
        state.currentFlightMode = ANGLE_MODE;
    }
    // Intermediate values could imply other modes or be ignored.

    // 3. Calculate setpoints for Roll, Pitch, Yaw, and Throttle based on flight mode and mapped channels.
    uint16_t rollChannelValue = state.receiverChannels[_settings.channelMapping.channel[ROLL]];
    uint16_t pitchChannelValue = state.receiverChannels[_settings.channelMapping.channel[PITCH]];
    uint16_t yawChannelValue = state.receiverChannels[_settings.channelMapping.channel[YAW]];
    uint16_t throttleChannelValue = state.receiverChannels[_settings.channelMapping.channel[THROTTLE]];

    if (state.currentFlightMode == ANGLE_MODE)
    {
        // In ANGLE_MODE, stick input directly maps to a target angle (degrees).
        state.setpoints.roll = map(rollChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxAngleRollPitch, _settings.rates.maxAngleRollPitch);
        state.setpoints.pitch = map(pitchChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxAngleRollPitch, _settings.rates.maxAngleRollPitch);
    }
    else // ACRO_MODE (or any other rate-based mode)
    {
        // In ACRO_MODE, stick input maps to a target rotation rate (degrees/second).
        state.setpoints.roll = map(rollChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxRateRollPitch, _settings.rates.maxRateRollPitch);
        state.setpoints.pitch = map(pitchChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxRateRollPitch, _settings.rates.maxRateRollPitch);
    }

    // Yaw control is typically always rate-based, regardless of flight mode.
    state.setpoints.yaw = map(yawChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxRateYaw, _settings.rates.maxRateYaw);

    // Map throttle input from receiver's raw range to the DShot throttle range.
    state.throttle = map(throttleChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
}
