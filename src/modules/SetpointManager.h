// SetpointManager.h
//
// This file defines the SetpointManager class, which is responsible for
// calculating the target setpoints (desired roll, pitch, yaw rates/angles)
// based on the pilot's receiver input and the currently active flight mode.
// It translates raw stick inputs into control targets for the PID controllers.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef SETPOINT_MANAGER_MODULE_H
#define SETPOINT_MANAGER_MODULE_H

#include "src/config/FlightState.h"
#include "src/config/config.h" // Required for ReceiverSettings and RateSettings
#include "src/hardware/receiver/ReceiverInterface.h"

// Calculates the target setpoints (desired roll, pitch, yaw rates/angles)
// based on the pilot's receiver input and the currently active flight mode.
// This module translates raw stick inputs into control targets for the PID controllers.
class SetpointManager
{
public:
    // Constructor: Initializes the SetpointManager with references to the receiver and settings.
    SetpointManager(ReceiverInterface &receiver, const ReceiverSettings &receiverSettings, const FlightRateSettings &rateSettings);

    // Performs one cycle of setpoint calculation.
    // It reads the relevant receiver channels and the current flight mode from the FlightState,
    // applies scaling and mixing logic, and updates the `setpointRoll`, `setpointPitch`,
    // and `setpointYaw` values within the FlightState.
    void update(FlightState &state);

private:
    ReceiverInterface &_receiver;              // Reference to the active RC receiver
    const ReceiverSettings &_receiverSettings; // Reference to receiver-specific settings
    const FlightRateSettings &_rateSettings;         // Reference to rate-specific settings

    // Private helper methods for setpoint calculations
    float _calculateSetpoint(uint16_t channelValue, float maxRateOrAngle) const;
    float _calculateThrottle(uint16_t throttleChannelValue) const;
};

#endif // SETPOINT_MANAGER_MODULE_H
