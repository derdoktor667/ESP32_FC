#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include "src/config/FlightState.h"
#include "src/config/config.h"

// Forward declaration
class FlightController;

// Enum to represent the main operating modes of the serial interface.
enum class OperatingMode
{
    FLIGHT, // Default mode, no interactive CLI or API active.
    CLI,    // Human-readable command-line interface.
    API     // Machine-readable JSON-based API for programmatic clients.
};

class CommunicationManager
{
public:
    // Constructor: Takes a reference to the flight controller to send commands to it.
    CommunicationManager(FlightController* fc);

    // Initializes the serial port.
    void begin();

    // Main update loop, called repeatedly. Handles incoming serial data.
    void update(const FlightState &state);

private:
    // --- State ---
    OperatingMode _currentMode = OperatingMode::FLIGHT;
    FlightController* _fc; // Pointer to the main flight controller
    unsigned long _lastSerialLogTime = 0;

    // --- Command Handling ---
    void _handleSerialInput(const FlightState &state);
    void _executeCommand(String command, const FlightState &state, bool isApiMode);

    // --- Command Implementations ---
    void _handleGetCommand(String args, bool isApiMode);
    void _handleSetCommand(String args, bool isApiMode);
    void _handleDumpCommand();
    void _handleDumpJsonCommand();
    void _printPidSettings(bool isApiMode);
    void _printRatesSettings(bool isApiMode);
    void _printFilterSettings(bool isApiMode);
    void _printReceiverSettings(bool isApiMode);
    void _printImuSettings(bool isApiMode);
    void _printMotorSettings(bool isApiMode);

    // --- Logging & Output ---
    void _printCliHelp();
    void _printFlightStatus(const FlightState &state);
};

#endif // COMMUNICATION_MANAGER_H
