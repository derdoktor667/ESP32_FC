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
    // First, determine the current flight mode.
    uint16_t flightModeChannel = state.receiverChannels[CHANNEL_FLIGHT_MODE];
    if (flightModeChannel < 1200)
    {
        state.currentFlightMode = ACRO_MODE;
    }
    else if (flightModeChannel > 1800)
    {
        state.currentFlightMode = ANGLE_MODE;
    }

    // Second, calculate setpoints based on the flight mode.
    if (state.currentFlightMode == ANGLE_MODE)
    {
        // In ANGLE_MODE, the stick input maps to a target angle.
        state.setpoints.roll = map(state.receiverChannels[CHANNEL_ROLL], _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxAngleRollPitch, _settings.rates.maxAngleRollPitch);
        state.setpoints.pitch = map(state.receiverChannels[CHANNEL_PITCH], _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxAngleRollPitch, _settings.rates.maxAngleRollPitch);
    }
    else // ACRO_MODE
    {
        // In ACRO_MODE, the stick input maps to a target rotation rate.
        state.setpoints.roll = map(state.receiverChannels[CHANNEL_ROLL], _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxRateRollPitch, _settings.rates.maxRateRollPitch);
        state.setpoints.pitch = map(state.receiverChannels[CHANNEL_PITCH], _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxRateRollPitch, _settings.rates.maxRateRollPitch);
    }

    // Yaw is always rate-controlled.
    state.setpoints.yaw = map(state.receiverChannels[CHANNEL_YAW], _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, -_settings.rates.maxRateYaw, _settings.rates.maxRateYaw);

    // Map throttle from receiver range to DShot range
    state.throttle = map(state.receiverChannels[CHANNEL_THROTTLE], _settings.receiver.ibusMinValue, _settings.receiver.ibusMaxValue, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
}
