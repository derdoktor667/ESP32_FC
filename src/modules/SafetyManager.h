// SafetyManager.h
//
// This file defines the SafetyManager class, which is responsible for handling
// the arming, disarming, and failsafe logic of the flight controller. It monitors
// receiver inputs and sensor states to ensure safe operation of the drone.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef SAFETY_MANAGER_MODULE_H
#define SAFETY_MANAGER_MODULE_H

#include "src/config/FlightState.h"
#include "src/hardware/receiver/ReceiverInterface.h"

// Manages the arming, disarming, and failsafe logic of the flight controller.
// This module continuously monitors receiver input for specific safety-related
// commands (e.g., arm/disarm switch, failsafe switch) and updates the
// `isArmed` and `isFailsafeActive` flags within the FlightState.
class SafetyManager
{
public:
    // Constructor: Initializes the SafetyManager with references to the receiver and settings.
    SafetyManager(ReceiverInterface &receiver, const FlightControllerSettings &settings);

    // Performs one cycle of safety checks.
    // It reads the relevant receiver channels, evaluates arming/disarming conditions,
    // and checks for failsafe activation. The FlightState is updated accordingly.
    void update(FlightState &state);

private:
    ReceiverInterface &_receiver;              // Reference to the active RC receiver
    const FlightControllerSettings &_settings; // Reference to global flight controller settings
};

#endif // SAFETY_MANAGER_MODULE_H
