#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "FlightState.h"
#include "ReceiverInterface.h"
#include "modules/AttitudeEstimator.h"
#include "modules/SafetyManager.h"
#include "modules/SetpointManager.h"
#include "modules/PidProcessor.h"
#include "modules/MotorMixer.h"
#include <ESP32_MPU6050.h>
#include <DShotRMT.h>

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

private:
    // --- Core State ---
    FlightState _state;

    // --- Hardware Objects ---
    ESP32_MPU6050 _imu;
    ReceiverInterface *_receiver;
    DShotRMT _motor1, _motor2, _motor3, _motor4;

    // --- Processing Modules ---
    AttitudeEstimator _attitudeEstimator;
    SafetyManager *_safetyManager;
    SetpointManager *_setpointManager;
    PidProcessor _pidProcessor;
    MotorMixer _motorMixer;

    // --- Timing ---
    unsigned long _lastLogTime = 0;

    // Destructor to clean up dynamically allocated objects
    public: ~FlightController();
};

#endif // FLIGHT_CONTROLLER_H
