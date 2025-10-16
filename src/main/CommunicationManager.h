// CommunicationManager.h
//
// This file defines the CommunicationManager class, which handles all serial
// communication (CLI/API) and logging for the ESP32 Flight Controller. It
// manages different operating modes (FLIGHT, CLI, API) and processes incoming
// commands and outgoing flight status data.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef COMMUNICATION_MANAGER_H
#define COMMUNICATION_MANAGER_H

#include "src/config/FlightState.h"
#include "src/config/config.h"
#include "src/config/settings.h"
#include <Arduino.h>

// Forward declarations for ESP system functions
extern "C" {
    uint32_t esp_get_free_heap_size(void);
    uint32_t esp_get_minimum_free_heap_size(void);
}

// Forward declaration to break circular dependency
class FlightController;

// Manages all serial communication (CLI/API) and logging.
// This class handles incoming commands from the serial port (both human-readable CLI
// and machine-readable JSON API) and outputs flight status data.
class CommunicationManager
{
public:
    // Constructor: Initializes the CommunicationManager with a pointer to the FlightController.
    CommunicationManager(FlightController* fc);

    // Initializes serial communication.
    void begin();

    // Updates the communication manager, handling serial input and output.
    // @param state The current FlightState, used for logging and command processing.
    void update(const FlightState &state);

private:
    // Defines the operating mode of the serial interface.
    enum class OperatingMode
    {
        FLIGHT, // Default mode, no CLI/API active, main flight loop runs.
        CLI,    // Command Line Interface mode.
        API     // Machine-readable JSON API mode.
    };

    FlightController* _fc; // Pointer to the FlightController instance
    OperatingMode _currentMode = OperatingMode::FLIGHT; // Current operating mode
    unsigned long _lastSerialLogTime = 0; // Timestamp of the last serial log output
    bool _isSendingSettings = false; // Flag to prevent live_data stream during settings dump

public:
    enum class SetResult {
        SUCCESS,
        INVALID_FORMAT,
        UNKNOWN_PARAMETER,
        INVALID_VALUE,
        OUT_OF_RANGE
    };

    // Private helper methods for command processing
    void _handleSerialInput();
    void _executeCommand(String command, bool isApiMode);
    void _handleGetCommand(String args, bool isApiMode);
    void _handleSetCommand(String args, bool isApiMode);
    void _handleDumpCommand();
    void _handleDumpJsonCommand();
    void _printCliHelp();
    void _printFlightStatus(const FlightState &state);
    void _printGetResponse(const String& param, const String& value, bool isApiMode, bool isString);
    void _printSetResponse(const String& param, const String& value, SetResult result, bool isApiMode, bool isString = false, const String& expected = "");
    void _handleStatusCommand();
    void _handleVersionCommand(); // New method for version command
};

#endif // COMMUNICATION_MANAGER_H
