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
    unsigned long _lastApiPingTime = 0; // Timestamp of the last API ping command
    unsigned long _lastSerialLogTime = 0; // Timestamp of the last serial log output

    static constexpr unsigned long API_MODE_TIMEOUT_MS = 2000; // API mode timeout in milliseconds

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
    void _printSetResponse(const String& param, const String& value, bool success, bool isApiMode, bool isString);
    void _handleStatusCommand();
    void _handleVersionCommand(); // New method for version command
};
