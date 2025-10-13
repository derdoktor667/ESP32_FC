#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "src/config/FlightState.h"
#include "src/hardware/receiver/ReceiverInterface.h"
#include "src/hardware/imu/ImuInterface.h"
#include "src/hardware/imu/Mpu6050Imu.h"
#include "src/modules/AttitudeEstimator.h"
#include "src/modules/SafetyManager.h"
#include "src/modules/SetpointManager.h"
#include "src/modules/PidProcessor.h"
#include "src/modules/MotorMixer.h"
// #include "src/main/CommunicationManager.h" // Include CommunicationManager - REMOVED to break circular dependency
#include <DShotRMT.h>

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

    // Sets the CommunicationManager instance.
    void setCommunicationManager(CommunicationManager* comms);

    // Requests IMU calibration.
    void requestImuCalibration();

    // The public state of the flight controller, readable by other modules.
    FlightState state;

private:
    // --- Hardware Objects ---
    ImuInterface *_imuInterface = nullptr; // Pointer to the active IMU interface
    ReceiverInterface *_receiver;
    DShotRMT *_motor1 = nullptr, *_motor2 = nullptr, *_motor3 = nullptr, *_motor4 = nullptr;

    // --- Processing Modules ---
    AttitudeEstimator _attitudeEstimator;
    SafetyManager *_safetyManager;
    SetpointManager *_setpointManager;
    PidProcessor _pidProcessor;
    MotorMixer *_motorMixer = nullptr;

    // --- Communication Manager ---
    CommunicationManager* _comms = nullptr; // Communication Manager instance (now a pointer)

    // Destructor to clean up dynamically allocated objects
    public: ~FlightController();
};

#endif // FLIGHT_CONTROLLER_H
