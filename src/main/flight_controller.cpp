#include "src/main/flight_controller.h"
#include "src/config/config.h"
#include "src/config/settings.h"
#include "src/hardware/receiver/IbusReceiver.h"
#include "src/hardware/receiver/PpmReceiver.h"
#include "src/main/CommunicationManager.h" // Include full definition of CommunicationManager
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

// Initializes all components of the flight controller in the correct sequence.
void FlightController::setCommunicationManager(CommunicationManager* comms)
{
    _comms = comms;
}

void FlightController::initialize()
{
    // 1. Load persistent settings from flash memory first.
    loadSettings();

    // 2. Dynamically allocate hardware drivers based on loaded settings.
    _motor1 = new DShotRMT(ESC_PIN_FRONT_RIGHT, settings.dshotMode, false);
    _motor2 = new DShotRMT(ESC_PIN_FRONT_LEFT, settings.dshotMode, false);
    _motor3 = new DShotRMT(ESC_PIN_REAR_RIGHT, settings.dshotMode, false);
    _motor4 = new DShotRMT(ESC_PIN_REAR_LEFT, settings.dshotMode, false);

    Serial.print("Initializing Receiver Protocol: ");
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
        Serial.println("Unknown! Halting.");
        while (INFINITE_LOOP_CONDITION);
    }
    _receiver->begin();
    Serial.println("Receiver initialized.");

    Serial.print("Initializing IMU Protocol: ");
    switch (settings.imuProtocol)
    {
    case IMU_MPU6050:
        Serial.println("MPU6050");
        _imuInterface = new Mpu6050Imu();
        break;
    default:
        Serial.println("Unknown! Halting.");
        while (INFINITE_LOOP_CONDITION);
    }
    _imuInterface->begin();
    Serial.println("IMU initialized.");

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
    // Update CommunicationManager first to handle any incoming commands
    _comms->update(state);

    // When logging is enabled (API mode), we skip the main flight logic
    // to prevent stack overflows and ensure stable communication.
    if (!settings.enableLogging)
    {
        // The flight control process follows a strict sequence (a "pipeline").
        for (int i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
        {
            state.receiverChannels[i] = _receiver->getChannel(i);
        }
        _attitudeEstimator.update(state);
        _safetyManager->update(state);
        _setpointManager->update(state);
        _pidProcessor.update(state);
        _motorMixer->apply(state);
    }
    else
    {
    // Even when not running the full pipeline, we must update the attitude
    // so a connected client can get live data.
    _attitudeEstimator.update(state);
    }
}

void FlightController::requestImuCalibration()
{
    _attitudeEstimator.calibrate();
}