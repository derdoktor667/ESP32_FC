#include "flight_controller.h"
#include "config.h"
#include "settings.h"
#include "serial_logger.h"
#include "cli.h"
#include "IbusReceiver.h"
#include "PpmReceiver.h"
#include <Arduino.h>

// Constructor: Initializes hardware drivers and processing modules in a safe order.
FlightController::FlightController()
    // Initialize DShot motor drivers with their respective pins and protocol.
    : _motor1(ESC_PIN_FRONT_RIGHT, DSHOT300, false),
      _motor2(ESC_PIN_FRONT_LEFT, DSHOT300, false),
      _motor3(ESC_PIN_REAR_RIGHT, DSHOT300, false),
      _motor4(ESC_PIN_REAR_LEFT, DSHOT300, false),
      // Initialize modules that have no external dependencies.
      _pidProcessor(settings),
      _motorMixer(_motor1, _motor2, _motor3, _motor4, settings)
{
    // Pointers to interfaces and dependent modules are initialized to nullptr.
    // They will be dynamically allocated in the initialize() method after
    // settings have been loaded.
    _imuInterface = nullptr;
    _receiver = nullptr;
    _safetyManager = nullptr;
    _setpointManager = nullptr;
}

// Destructor: Ensures all dynamically allocated objects are properly deleted.
FlightController::~FlightController()
{
    delete _receiver;
    delete _safetyManager;
    delete _setpointManager;
    delete _imuInterface;
}

// Initializes all components of the flight controller in the correct sequence.
void FlightController::initialize()
{
    // 1. Load persistent settings from flash memory first.
    // This is critical as all subsequent initializations depend on these settings.
    loadSettings();

    // 2. Initialize the RC receiver based on the selected protocol.
    // This is a factory pattern: create the concrete receiver object.
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
        while (1); // Halt on critical configuration error.
    }
    _receiver->begin();
    Serial.println("Receiver initialized.");

    // 3. Initialize the IMU sensor based on the selected protocol.
    // Factory pattern for the IMU object.
    Serial.print("Initializing IMU Protocol: ");
    switch (settings.imuProtocol)
    {
    case IMU_MPU6050:
        Serial.println("MPU6050");
        _imuInterface = new Mpu6050Imu();
        break;
    default:
        Serial.println("Unknown! Halting.");
        while (1); // Halt on critical configuration error.
    }
    _imuInterface->begin();
    Serial.println("IMU initialized.");

    // 4. Initialize all processing modules that have dependencies.
    // These modules require the receiver and/or IMU to be available.
    _safetyManager = new SafetyManager(*_receiver, settings);
    _setpointManager = new SetpointManager(*_receiver, settings);
    _attitudeEstimator.init(*_imuInterface, settings);

    // 5. Start the remaining components.
    _attitudeEstimator.begin();
    _motorMixer.begin();
}

// This is the main flight control loop, executed repeatedly.
void FlightController::runLoop()
{
    // The flight control process follows a strict sequence (a "pipeline").

    // 1. Read Pilot Input: Get the latest commands from the RC receiver.
    for (int i = 0; i < RECEIVER_CHANNEL_COUNT; i++) {
        _state.receiverChannels[i] = _receiver->getChannel(i);
    }

    // 2. Estimate Attitude: Process IMU data to calculate the current orientation.
    _attitudeEstimator.update(_state);

    // 3. Update Safety Status: Check for arming, disarming, and failsafe conditions.
    _safetyManager->update(_state);

    // 4. Determine Setpoints: Calculate the target roll, pitch, and yaw rates.
    _setpointManager->update(_state);

    // 5. Calculate PID Corrections: Compute the necessary adjustments to reach the setpoints.
    _pidProcessor.update(_state);

    // 6. Mix and Apply Motor Commands: Combine PID outputs with throttle and send to motors.
    _motorMixer.apply(_state);

    // 7. Handle Communication: Process incoming CLI commands and send log data.
    if (millis() - _lastSerialLogTime >= settings.printIntervalMs)
    {
        printFlightStatus(_state);
        _lastSerialLogTime = millis();
    }
    CliCommand cliCmd = handleSerialCli(_state);
    if (cliCmd == CliCommand::CALIBRATE_IMU)
    {
        _attitudeEstimator.calibrate();
    }
}
