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
extern "C"
{
    uint32_t esp_get_free_heap_size(void);
    uint32_t esp_get_minimum_free_heap_size(void);
}

// Forward declaration to break circular dependency
class FlightController;

enum class SetResult
{
    SUCCESS,
    INVALID_FORMAT,
    UNKNOWN_PARAMETER,
    INVALID_VALUE,
    OUT_OF_RANGE
};

// Manages all serial communication (CLI/API) and logging.
// This class handles incoming commands from the serial port (both human-readable CLI
// and machine-readable JSON API) and outputs flight status data.
enum class SettingType
{
    FLOAT,
    INT,
    UINT8, // Added for uint8_t parameters like filter stages
    UINT16,
    BOOL,
    STRING,
    ENUM_IBUS_PROTOCOL,
    ENUM_IMU_PROTOCOL,
    ENUM_IMU_ROTATION, // New enum type for IMU rotation
    ENUM_DSHOT_MODE,
    ENUM_LPF_BANDWIDTH,
    ENUM_RX_CHANNEL_MAP
};

struct Setting
{
    const char *name;
    SettingType type;
    void *value;
    const float scaleFactor;
};

class CommunicationManager
{
public:
    // Constructor: Initializes the CommunicationManager with a pointer to the FlightController.
    CommunicationManager(FlightController *fc);

    // Initializes serial communication.
    void initializeCommunication();

    // Updates the communication manager, handling serial input and output.
    void processCommunication();

private:
    // Defines the operating mode of the serial interface.
    enum class OperatingMode
    {
        FLIGHT, // Default mode, no CLI/API active, main flight loop runs.
        CLI,    // Command Line Interface mode.
        API     // Machine-readable JSON API mode.
    };

    FlightController *_fc;                              // Pointer to the FlightController instance
    OperatingMode _currentMode = OperatingMode::FLIGHT; // Current operating mode
    unsigned long _lastSerialLogTime = 0;               // Timestamp of the last serial log output
    bool _isSendingSettings = false;                    // Flag to prevent live_data stream during settings dump

    static const Setting settingsRegistry[];
    static const int numSettings;

    static constexpr float DEFAULT_SCALE_FACTOR = 1.0f;

    // JSON Document Sizes
    static constexpr size_t JSON_DOC_SMALL_SIZE = 64;
    static constexpr size_t JSON_DOC_MEDIUM_SIZE = 256;
    static constexpr size_t JSON_DOC_LARGE_SIZE = 512;
    static constexpr size_t JSON_DOC_XLARGE_SIZE = 4096;

    // API Error Messages
    static constexpr const char *API_ERROR_UNKNOWN_COMMAND = "{\"error\":\"Unknown command\"}";
    static constexpr const char *API_ERROR_UNKNOWN_PARAMETER_GET = "{\"error\":\"Unknown parameter for get\"}";
    static constexpr const char *API_MODE_ACTIVATED_JSON = "{\"status\":\"api_mode_activated\"}";

    // Helper functions for string conversion
    String _getReceiverProtocolString(ReceiverProtocol protocol) const;
    String _getImuProtocolString(ImuProtocol protocol) const;
    String _getLpfBandwidthString(LpfBandwidth bandwidth) const;
    String _getFlightControlInputString(FlightControlInput input) const;
    String _getDShotModeString(dshot_mode_t mode) const;
    String _getImuRotationString(ImuRotation rotation) const; // New declaration
    String _getBoolString(bool value) const;
    String _getUint8String(uint8_t value) const;

    // Helper functions for printing responses
    void _printGetResponse(const String &param, const String &value, bool isApiMode, bool isString) const;
    void _printSetResponse(const String &param, const String &value, SetResult result, bool isApiMode, bool isString, const String &expected) const;

    static constexpr int RX_MAP_PREFIX_LENGTH = 7; // Length of "rx.map."

    // Helper functions for parsing and validating setting values
    SetResult _parseAndValidateFloat(const String &valueStr, float &outValue, float scaleFactor, String &expectedValue) const;
    SetResult _parseAndValidateInt(const String &valueStr, int &outValue, String &expectedValue) const; // New declaration
    SetResult _parseAndValidateUint16(const String &valueStr, uint16_t &outValue, String &expectedValue) const;
    SetResult _parseAndValidateReceiverProtocol(const String &valueStr, ReceiverProtocol &outValue, String &expectedValue) const;
    SetResult _parseAndValidateImuProtocol(const String &valueStr, ImuProtocol &outValue, String &expectedValue) const;
    SetResult _parseAndValidateLpfBandwidth(const String &valueStr, LpfBandwidth &outValue, String &expectedValue) const;
    SetResult _parseAndValidateImuRotation(const String &valueStr, ImuRotation &outValue, String &expectedValue) const; // New declaration
    SetResult _parseAndValidateDShotMode(const String &valueStr, dshot_mode_t &outValue, String &expectedValue) const;
    SetResult _parseAndValidateBool(const String &valueStr, bool &outValue, String &expectedValue) const;
    SetResult _parseAndValidateRxChannelMap(const String &param, const String &valueStr, int &outValue, String &expectedValue) const;

    // Private helper methods for command processing
    void _handleSerialInput();
    void _executeCommand(String command, bool isApiMode);
    void _handleSettingsCommand(String commandName, String commandArgs, bool isApiMode);
    void _handleSystemCommand(String commandName, bool isApiMode);
    void _handleUtilityCommand(String commandName, bool isApiMode);
    void _handleGetCommand(String args, bool isApiMode);
    void _handleSetCommand(String args, bool isApiMode);
    void _handleDumpCommand();
    void _handleDumpJsonCommand();
    void _printCliHelp();

    void _handleStatusCommand() const;
    void _handleVersionCommand() const;
    void _printFlightStatus() const;

    void _handleFlightModeInput(const String &input);

}; // Closing brace for CommunicationManager class

#endif // COMMUNICATION_MANAGER_H
