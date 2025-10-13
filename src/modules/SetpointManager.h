#ifndef SETPOINT_MANAGER_MODULE_H
#define SETPOINT_MANAGER_MODULE_H

#include "src/config/FlightState.h"
#include "src/hardware/receiver/ReceiverInterface.h"

// Calculates the target setpoints (desired roll, pitch, yaw rates/angles)
// based on the pilot's receiver input and the currently active flight mode.
// This module translates raw stick inputs into control targets for the PID controllers.
class SetpointManager
{
public:
    // Constructor: Initializes the SetpointManager with references to the receiver and settings.
    // @param receiver Reference to the active RC receiver interface.
    // @param settings Reference to the global flight controller settings.
    SetpointManager(ReceiverInterface &receiver, const FlightControllerSettings &settings);

    // Performs one cycle of setpoint calculation.
    // It reads the relevant receiver channels and the current flight mode from the FlightState,
    // applies scaling and mixing logic, and updates the `setpointRoll`, `setpointPitch`,
    // and `setpointYaw` values within the FlightState.
    // @param state Reference to the current FlightState to be updated with new setpoints.
    void update(FlightState &state);

private:
    ReceiverInterface &_receiver;              // Reference to the active RC receiver
    const FlightControllerSettings &_settings; // Reference to global flight controller settings
};

#endif // SETPOINT_MANAGER_MODULE_H
