#include "SetpointManager.h"
#include "../config.h"
#include "MotorMixer.h" // For DSHOT_MIN_THROTTLE and DSHOT_MAX_THROTTLE
#include <Arduino.h>

// Constructor: Initializes with references to the receiver and settings.
SetpointManager::SetpointManager(ReceiverInterface &receiver, const FlightControllerSettings &settings)
    : _receiver(receiver), _settings(settings)
{
}

// Calculates the target setpoints for roll, pitch, and yaw.
void SetpointManager::update(FlightState &state)
{
    // Read all raw channel values from the receiver
    for (int i = 0; i < RECEIVER_CHANNEL_COUNT; ++i)
    {
        state.receiverChannels[i] = _receiver.getChannel(i);
    }

    // First, determine the current flight mode using the mapped channel.
    uint16_t flightModeChannelValue = state.receiverChannels[_settings.channelMapping.channel[FLIGHT_MODE_SWITCH]];
    if (flightModeChannelValue < 1200)
    {
        state.currentFlightMode = ACRO_MODE;
    }
    else if (flightModeChannelValue > 1800)
    {
        state.currentFlightMode = ANGLE_MODE;
    }

    // Second, calculate setpoints based on the flight mode and mapped channels.
    uint16_t rollChannelValue = state.receiverChannels[_settings.channelMapping.channel[ROLL]];
    uint16_t pitchChannelValue = state.receiverChannels[_settings.channelMapping.channel[PITCH]];
    uint16_t yawChannelValue = state.receiverChannels[_settings.channelMapping.channel[YAW]];
    uint16_t throttleChannelValue = state.receiverChannels[_settings.channelMapping.channel[THROTTLE]];

    if (state.currentFlightMode == ANGLE_MODE)
    {
        // In ANGLE_MODE, the stick input maps to a target angle.
        state.setpoints.roll = map(rollChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxAngleRollPitch, _settings.rates.maxAngleRollPitch);
        state.setpoints.pitch = map(pitchChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxAngleRollPitch, _settings.rates.maxAngleRollPitch);
    }
    else // ACRO_MODE
    {
        // In ACRO_MODE, the stick input maps to a target rotation rate.
        state.setpoints.roll = map(rollChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxRateRollPitch, _settings.rates.maxRateRollPitch);
        state.setpoints.pitch = map(pitchChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxRateRollPitch, _settings.rates.maxRateRollPitch);
    }

    // Yaw is always rate-controlled.
    state.setpoints.yaw = map(yawChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxRateYaw, _settings.rates.maxRateYaw);

    // Map throttle from receiver range to DShot range
    state.throttle = map(throttleChannelValue, _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
}
