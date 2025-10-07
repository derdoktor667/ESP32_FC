#include "flight_controller.h"
#include "config.h"
#include "settings.h"
#include "serial_logger.h"
#include "cli.h"
#include "IbusReceiver.h"
#include "PpmReceiver.h"
#include <Arduino.h>

// Constructor: Initializes all hardware objects and processing modules.
FlightController::FlightController()
    // Initialize hardware objects
    : _imu(),
      _receiver(nullptr), // Initialize receiver pointer to nullptr
      _motor1(ESC_PIN_1, DSHOT300, false),
      _motor2(ESC_PIN_2, DSHOT300, false),
      _motor3(ESC_PIN_3, DSHOT300, false),
      _motor4(ESC_PIN_4, DSHOT300, false),
      // Initialize modules that don't depend on _receiver yet
      _attitudeEstimator(_imu, settings),
      _safetyManager(nullptr), // Initialize safetyManager pointer to nullptr
      _setpointManager(nullptr), // Initialize setpointManager pointer to nullptr
      _pidProcessor(settings),
      _motorMixer(_motor1, _motor2, _motor3, _motor4, settings)
{
}

// Destructor: Cleans up dynamically allocated objects.
FlightController::~FlightController()
{
    delete _receiver;
    delete _safetyManager;
    delete _setpointManager;
}

// Initializes the flight controller.
void FlightController::initialize()
{
    loadSettings(); // Load settings from flash

    // --- Receiver Initialization (Factory) ---
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
        while (1);
    }
    _receiver->begin();
    Serial.println("Receiver initialized.");

    // Now that _receiver is valid, create modules that depend on it.
    _safetyManager = new SafetyManager(*_receiver, settings);
    _setpointManager = new SetpointManager(*_receiver, settings);

    // --- Other Module Initializations ---
    _attitudeEstimator.begin();
    _motorMixer.begin();
}

// Main flight loop.
void FlightController::runLoop()
{
    // --- Read Inputs ---
    // Read all receiver channels into the state at once.
    for (int i = 0; i < 16; i++) {
        _state.receiverChannels[i] = _receiver->getChannel(i);
    }

    // --- Process Modules in Sequence ---
    _attitudeEstimator.update(_state);
    _safetyManager->update(_state);
    _setpointManager->update(_state);
    _pidProcessor.update(_state);
    _motorMixer.apply(_state);

    // --- Logging and CLI ---
    if (millis() - _lastLogTime >= settings.printIntervalMs)
    {
        printFlightStatus(_state);
        _lastLogTime = millis();
    }
    CliCommand cliCmd = handleSerialCli(_state);
    if (cliCmd == CliCommand::CALIBRATE_IMU)
    {
        _attitudeEstimator.calibrate();
    }
}
