#ifndef SAFETY_MANAGER_MODULE_H
#define SAFETY_MANAGER_MODULE_H

#include "../FlightState.h"
#include "../ReceiverInterface.h"

// Manages the arming, disarming, and failsafe logic.
//
// This class checks the receiver channels for safety-related commands
// and updates the flight state accordingly.
class SafetyManager
{
public:
    // Constructor.
    // - receiver: A reference to the active receiver.
    // - settings: A reference to the flight controller settings.
    SafetyManager(ReceiverInterface &receiver, const FlightControllerSettings &settings);

    // Performs one cycle of the safety checks.
    // Reads from the receiver and updates the isArmed and isFailsafeActive
    // flags in the FlightState.
    // - state: The current flight state to be updated.
    void update(FlightState &state);

private:
    ReceiverInterface &_receiver; // Reference to the receiver
    const FlightControllerSettings &_settings; // Reference to global settings
};

#endif // SAFETY_MANAGER_MODULE_H
