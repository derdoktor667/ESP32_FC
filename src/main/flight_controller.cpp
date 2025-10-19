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
#include "src/hardware/imu/Mpu6050Imu.h" // Required for Mpu6050Imu instantiation
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
    // _imuInterface = nullptr; // Handled by std::unique_ptr default constructor
    // _receiver = nullptr;     // Handled by std::unique_ptr default constructor
    // _safetyManager = nullptr; // Handled by std::unique_ptr default constructor
    // _setpointManager = nullptr; // Handled by std::unique_ptr default constructor
    // _motor1 = nullptr;       // Handled by std::unique_ptr default constructor
    // _motor2 = nullptr;       // Handled by std::unique_ptr default constructor
    // _motor3 = nullptr;       // Handled by std::unique_ptr default constructor
    // _motor4 = nullptr;       // Handled by std::unique_ptr default constructor
    // _motorMixer = nullptr;   // Handled by std::unique_ptr default constructor
}

// Sets the CommunicationManager instance. This is used to break a circular dependency.
void FlightController::setCommunicationManager(CommunicationManager *comms)
{
    _comms = comms;
}

// Initializes all components of the flight controller in the correct sequence.
void FlightController::initialize()
{
    // 1. Load persistent settings from flash memory first.
    loadSettings();

    // 2. Initialize hardware drivers.
    _initializeMotors();
    _initializeReceiver();
    _initializeImu();

    // 3. Initialize all processing modules that have dependencies.
    _initializeModules();
}

// This is the main flight control loop, executed repeatedly.
void FlightController::runLoop()
{
    // Start loop timer
    _loopTimer = micros();

    // When logging is enabled (API mode), we skip the main flight logic
    // to prevent stack overflows and ensure stable communication.
    // The attitude estimator is still updated to provide live data to clients.
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
    if (settings.enforceLoopTime && currentLoopTimeUs < TARGET_LOOP_TIME_US)
    {
        delayMicroseconds(TARGET_LOOP_TIME_US - currentLoopTimeUs);
    }
}

// Requests IMU calibration from the AttitudeEstimator module.
void FlightController::requestImuCalibration()
{
    _attitudeEstimator.calibrate();
}

// --- Private Helper Methods ---

void FlightController::_initializeMotors()
{
    // Motors are initialized with their respective pins and DShot mode.
    _motor1 = std::make_unique<DShotRMT>(ESC_PIN_FRONT_RIGHT, settings.dshotMode, false);
    _motor2 = std::make_unique<DShotRMT>(ESC_PIN_FRONT_LEFT, settings.dshotMode, false);
    _motor3 = std::make_unique<DShotRMT>(ESC_PIN_REAR_RIGHT, settings.dshotMode, false);
    _motor4 = std::make_unique<DShotRMT>(ESC_PIN_REAR_LEFT, settings.dshotMode, false);
    _motorMixer = std::make_unique<MotorMixer>(_motor1.get(), _motor2.get(), _motor3.get(), _motor4.get(), settings);
    _motorMixer->begin();
}

void FlightController::_initializeReceiver()
{
    Serial.print("INFO: Initializing Receiver Protocol: ");
    switch (settings.receiverProtocol)
    {
    case PROTOCOL_IBUS:
        Serial.println("iBUS");
        _receiver = std::make_unique<IbusReceiver>(Serial2, IBUS_RX_PIN);
        break;
    case PROTOCOL_PPM:
        Serial.println("PPM");
        _receiver = std::make_unique<PpmReceiver>(IBUS_RX_PIN);
        break;
    default:
        _haltOnError("ERROR: Unknown receiver protocol! Halting.");
    }
    _receiver->begin();
    Serial.println("INFO: Receiver initialized.");
}

void FlightController::_initializeImu()
{
    Serial.print("INFO: Initializing IMU Protocol: ");
    switch (settings.imuProtocol)
    {
    case IMU_MPU6050:
        Serial.println("MPU6050");
        _imuInterface = std::make_unique<Mpu6050Imu>(settings.imuLpfBandwidth, settings.imuRotation);
        break;
    default:
        _haltOnError("ERROR: Unknown IMU protocol! Halting.");
    }
    _imuInterface->begin();
    Serial.println("INFO: IMU initialized.");
}

void FlightController::_initializeModules()
{
    _safetyManager = std::make_unique<SafetyManager>(*_receiver, settings);
    _setpointManager = std::make_unique<SetpointManager>(*_receiver, settings);
    _attitudeEstimator.init(*_imuInterface, settings);
    _attitudeEstimator.begin();
}

void FlightController::_haltOnError(const char *message)
{
    Serial.println(message);
    while (INFINITE_LOOP_CONDITION)
        ; // Halt on critical error
}