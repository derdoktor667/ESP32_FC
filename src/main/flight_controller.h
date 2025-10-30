// flight_controller.h
//
// This file defines the FlightController class, the main orchestrator for the
// ESP32 Flight Controller firmware. It manages all hardware objects and
// processing modules, and controls the overall flight state and main loop.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "src/config/FlightState.h"
#include "src/hardware/receiver/ReceiverInterface.h"
#include "src/hardware/imu/ImuInterface.h"
#include "src/modules/AttitudeEstimator.h"
#include "src/modules/SafetyManager.h"
#include "src/modules/SetpointManager.h"
#include "src/modules/PidProcessor.h"
#include "src/modules/MotorMixer.h"
#include <DShotRMT.h>
#include <memory> // Required for std::unique_ptr

// Forward declaration to break circular dependency
class CommunicationManager;

// The main orchestrator for the flight controller.
//
// This class owns all the hardware objects and processing modules.
// It manages the main flight loop and the overall state.
class FlightController
{
public:
    FlightController();
    // Initializes all hardware and modules.
    void initialize();

    // Runs one iteration of the main flight loop.
    void runLoop();



    // Requests IMU calibration from the AttitudeEstimator module.
    void requestImuCalibration();

    // Getter for AttitudeEstimator to allow CommunicationManager to update notch filter state
    AttitudeEstimator &getAttitudeEstimator() { return _attitudeEstimator; }

    // The public state of the flight controller, readable by other modules.
    FlightState state;

private:
    // --- Private Helper Methods for Initialization ---
    void _initializeMotors();
    void _initializeReceiver();
    void _initializeImu();
    void _initializeModules();

    // --- Error Handling ---
    void _haltOnError(const char *message);

    // --- Hardware Objects ---
    std::unique_ptr<ImuInterface> _imuInterface; // Pointer to the active IMU interface
    std::unique_ptr<ReceiverInterface> _receiver;
    std::unique_ptr<DShotRMT> _motor1, _motor2, _motor3, _motor4;

    // --- Processing Modules ---
    AttitudeEstimator _attitudeEstimator;
    std::unique_ptr<SafetyManager> _safetyManager;
    std::unique_ptr<SetpointManager> _setpointManager;
    PidProcessor _pidProcessor;
    std::unique_ptr<MotorMixer> _motorMixer;


    // --- Loop Timing ---
    unsigned long _currentLoopStartTimeUs = 0;
};

#endif // FLIGHT_CONTROLLER_H
