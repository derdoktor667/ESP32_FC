#ifndef SETPOINT_MANAGER_MODULE_H
#define SETPOINT_MANAGER_MODULE_H

#include "../FlightState.h"
#include "../ReceiverInterface.h"

// Calculates the target setpoints based on receiver input and flight mode.
class SetpointManager
{
public:
    // Constructor.
    // - receiver: A reference to the active receiver.
    // - settings: A reference to the flight controller settings.
    SetpointManager(ReceiverInterface &receiver, const FlightControllerSettings &settings);

    // Performs one cycle of setpoint calculation.
    // Reads receiver channels and flight mode from the FlightState and updates
    // the setpoints within the state.
    // - state: The current flight state to be updated.
    void update(FlightState &state);

private:
    ReceiverInterface &_receiver; // Reference to the receiver
    const FlightControllerSettings &_settings; // Reference to global settings
};

#endif // SETPOINT_MANAGER_MODULE_H
