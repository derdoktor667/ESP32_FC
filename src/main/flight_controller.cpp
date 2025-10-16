// flight_controller.cpp
//
// This file implements the FlightController class, which orchestrates the
// flight control logic, manages hardware interactions, and processes sensor data.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "src/main/flight_controller.h"
#include "src/config/config.h"
#include "src/config/settings.h"
#include "src/hardware/receiver/IbusReceiver.h"
#include "src/hardware/receiver/PpmReceiver.h"
#include "src/main/CommunicationManager.h"
#include <Arduino.h>

// Constructor: Initializes hardware drivers and processing modules in a safe order.
FlightController::FlightController()
    // Initialize modules that have no external dependencies.
    : _pidProcessor(settings)
{
    // Pointers to interfaces and dependent modules are initialized to nullptr.
    // They will be dynamically allocated in the initialize() method after
    // settings have been loaded.
    _imuInterface = nullptr;
    _receiver = nullptr;
    _safetyManager = nullptr;
    _setpointManager = nullptr;
    _motor1 = nullptr;
    _motor2 = nullptr;
    _motor3 = nullptr;
    _motor4 = nullptr;
    _motorMixer = nullptr;
}

// Destructor: Ensures all dynamically allocated objects are properly deleted.
FlightController::~FlightController()
{
    delete _receiver;
    delete _safetyManager;
    delete _setpointManager;
    delete _imuInterface;
    delete _motor1;
    delete _motor2;
    delete _motor3;
    delete _motor4;
    delete _motorMixer;
}

// Sets the CommunicationManager instance. This is used to break a circular dependency.
void FlightController::setCommunicationManager(CommunicationManager* comms)
{
    _comms = comms;
}

// Initializes all components of the flight controller in the correct sequence.
void FlightController::initialize()
{
    // 1. Load persistent settings from flash memory first.
    loadSettings();

    // 2. Dynamically allocate hardware drivers based on loaded settings.
    // Motors are initialized with their respective pins and DShot mode.
    _motor1 = new DShotRMT(ESC_PIN_FRONT_RIGHT, settings.dshotMode, false);
    _motor2 = new DShotRMT(ESC_PIN_FRONT_LEFT, settings.dshotMode, false);
    _motor3 = new DShotRMT(ESC_PIN_REAR_RIGHT, settings.dshotMode, false);
    _motor4 = new DShotRMT(ESC_PIN_REAR_LEFT, settings.dshotMode, false);

    Serial.print("INFO: Initializing Receiver Protocol: ");
    switch (settings.receiverProtocol)
    {
    case PROTOCOL_IBUS:
        Serial.println("iBUS");
        _receiver = new IbusReceiver(Serial2, IBUS_RX_PIN);
        break;
    case PROTOCOL_PPM:
        Serial.println("PPM");
        _receiver = new PpmReceiver(IBUS_RX_PIN);
        break;
    default:
        Serial.println("ERROR: Unknown receiver protocol! Halting.");
        while (INFINITE_LOOP_CONDITION); // Halt on critical error
    }
    _receiver->begin();
    Serial.println("INFO: Receiver initialized.");

    Serial.print("INFO: Initializing IMU Protocol: ");
    switch (settings.imuProtocol)
    {
    case IMU_MPU6050:
        Serial.println("MPU6050");
        _imuInterface = new Mpu6050Imu();
        break;
    default:
        Serial.println("ERROR: Unknown IMU protocol! Halting.");
        while (INFINITE_LOOP_CONDITION); // Halt on critical error
    }
    _imuInterface->begin();
    Serial.println("INFO: IMU initialized.");

    // 3. Initialize all processing modules that have dependencies.
    _motorMixer = new MotorMixer(_motor1, _motor2, _motor3, _motor4, settings);
    _safetyManager = new SafetyManager(*_receiver, settings);
    _setpointManager = new SetpointManager(*_receiver, settings);
    _attitudeEstimator.init(*_imuInterface, settings);
    _motorMixer->begin();

    // 4. Start the remaining components.
    _attitudeEstimator.begin();
}

// This is the main flight control loop, executed repeatedly.
void FlightController::runLoop()
{
    // Start loop timer
    _loopTimer = micros();

    // Update CommunicationManager first to handle any incoming commands
    _comms->update(state);

    // When logging is enabled (API mode), we skip the main flight logic
    // to prevent stack overflows and ensure stable communication.
    if (!settings.enableLogging)
    {
        // Read raw receiver channel values into the flight state.
        for (int i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
        {
            state.receiverChannels[i] = _receiver->getChannel(i);
        }
        // Update attitude estimation based on IMU data.
        _attitudeEstimator.update(state);
        // Check and update safety status (arming, failsafe).
        _safetyManager->update(state);
        // Calculate target setpoints from receiver input.
        _setpointManager->update(state);
        // Execute PID control loops and get motor corrections.
        _pidProcessor.update(state);
        // Mix PID outputs and apply to motors.
        _motorMixer->apply(state);
    }
    else
    {
    // Even when not running the full pipeline, we must update the attitude
    // so a connected client can get live data (e.g., for a ground station).
    _attitudeEstimator.update(state);
    }

    // Calculate actual loop time
    unsigned long currentLoopTimeUs = micros() - _loopTimer;
    state.loopTimeUs = currentLoopTimeUs; // Store actual loop time in state

    // Enforce target loop time
    if (currentLoopTimeUs < TARGET_LOOP_TIME_US) {
        delayMicroseconds(TARGET_LOOP_TIME_US - currentLoopTimeUs);
    }
}

// Requests IMU calibration from the AttitudeEstimator module.
void FlightController::requestImuCalibration()
{
    _attitudeEstimator.calibrate();
}