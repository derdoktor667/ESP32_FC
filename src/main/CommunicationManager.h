#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include <Arduino.h>
#include "src/config/FlightState.h"
#include "src/config/settings.h"
// #include "src/utils/cli/CliCommandProcessor.h" // Include the new command processor - REMOVED to break circular dependency

// Forward declarations to break circular dependencies
class FlightController;
class CliCommandProcessor;

// Manages all serial communication for the flight controller.
// This includes handling CLI commands, API requests, and streaming live data.
class CommunicationManager
{
public:
    // Defines the operating modes of the serial interface.
    enum class OperatingMode
    {
        FLIGHT, // Default mode, silent, no serial interaction
        CLI,    // Command Line Interface mode for human interaction
        API     // Machine-readable API mode for programmatic interaction
    };

    // Constructor: Initializes the CommunicationManager with a pointer to the FlightController.
    // @param fc Pointer to the FlightController instance.
    CommunicationManager(FlightController* fc);

    // Initializes serial communication.
    void begin();

    // Updates the communication manager, handling incoming serial data
    // and streaming live data if in API mode.
    // @param state The current FlightState to be used for live data streaming.
    void update(const FlightState &state);

    // Streams live flight data as a JSON object to the serial port.
    // This method is called periodically when in API mode.
    // @param state The current FlightState containing the data to be streamed.
    void _printFlightStatus(const FlightState &state);

private:
    FlightController* _fc;                     // Pointer to the FlightController instance
    OperatingMode _currentMode = OperatingMode::FLIGHT; // Current operating mode
    unsigned long _lastSerialLogTime = 0;      // Timestamp of the last serial log output

    CliCommandProcessor* _cliCommandProcessor;  // Instance of the CLI command processor (now a pointer)

    // Handles incoming serial data, parsing commands and managing mode changes.
    // @param state The current FlightState, used for command execution.
    void _handleSerialInput(const FlightState &state);

    // Executes a given command based on the current operating mode.
    // @param command The full command string received.
    // @param state The current FlightState.
    // @param isApiMode True if the command originated from API mode.
    void _executeCommand(String command, const FlightState &state, bool isApiMode);

    // --- Private Helper Functions for Command Implementations ---
    void _printCliHelp();
    void _printPidSettings(bool isApiMode);
    void _printRatesSettings(bool isApiMode);
    void _printFilterSettings(bool isApiMode);
    void _printReceiverSettings(bool isApiMode);
    void _printImuSettings(bool isApiMode);
    void _printMotorSettings(bool isApiMode);
    void _handleGetCommand(String args, bool isApiMode);
    void _handleSetCommand(String args, bool isApiMode);
    void _handleDumpCommand();
    void _handleDumpJsonCommand();
};

#endif // COMMUNICATION_MANAGER_H
