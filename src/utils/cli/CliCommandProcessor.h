#ifndef CLI_COMMAND_PROCESSOR_H
#define CLI_COMMAND_PROCESSOR_H

#include <Arduino.h>
#include "src/config/config.h"
#include "src/config/settings.h"
#include "src/config/FlightState.h"
#include "src/main/flight_controller.h" // For FlightController pointer

// Forward declaration to avoid circular dependency with FlightController
class FlightController;

// Manages the parsing and execution of CLI and API commands.
// This class encapsulates the logic for handling various commands
// related to getting, setting, and dumping flight controller settings,
// as well as triggering actions like IMU calibration.
class CliCommandProcessor
{
public:
    // Constructor: Initializes the command processor with a pointer to the
    // FlightController instance and a reference to the global settings.
    // @param fc Pointer to the FlightController instance.
    // @param settings Reference to the global FlightControllerSettings.
    CliCommandProcessor(FlightController* fc, FlightControllerSettings& settings);

    // Handles 'get' commands, retrieving and printing the value of a specified parameter.
    // @param args The arguments for the 'get' command (e.g., "pid.roll.kp").
    // @param isApiMode True if the command is from API mode, false for CLI mode.
    void handleGetCommand(String args, bool isApiMode);

    // Handles 'set' commands, updating the value of a specified parameter.
    // @param args The arguments for the 'set' command (e.g., "pid.roll.kp 1.2").
    // @param isApiMode True if the command is from API mode, false for CLI mode.
    void handleSetCommand(String args, bool isApiMode);

    // Handles the 'dump' command, printing all current settings in a human-readable format.
    void handleDumpCommand();

    // Handles the 'get_settings' command, printing all current settings as a JSON object.
    void handleDumpJsonCommand();

    // Prints the CLI help message to the serial output.
    void printCliHelp();

    // Triggers IMU calibration via the FlightController.
    void triggerImuCalibration();

private:
    FlightController* _fc;                     // Pointer to the FlightController instance
    FlightControllerSettings& _settings;       // Reference to the global settings

    // Helper functions for printing specific categories of settings
    void _printPidSettings(bool isApiMode);
    void _printRatesSettings(bool isApiMode);
    void _printFilterSettings(bool isApiMode);
    void _printReceiverSettings(bool isApiMode);
    void _printImuSettings(bool isApiMode);
    void _printMotorSettings(bool isApiMode);

    // Helper functions for string conversions (can be moved to a utility if needed elsewhere)
    static String getReceiverProtocolString(ReceiverProtocol protocol);
    static String getImuProtocolString(ImuProtocol protocol);
    static String getFlightControlInputString(FlightControlInput input);
    static String getDShotModeString(dshot_mode_t mode);
};

#endif // CLI_COMMAND_PROCESSOR_H
