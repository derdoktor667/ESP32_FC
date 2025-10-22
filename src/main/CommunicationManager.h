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
#include "src/config/MspCommands.h"
#include <Arduino.h>
#include <MspParser.h>

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

// Manages all serial communication (CLI/MSP API) and logging.
// This class handles incoming commands from the serial port (both human-readable CLI
// and machine-readable MSP API) and outputs flight status data.
enum class SettingType
{
    FLOAT,
    INT,
    UINT8, // Added for uint8_t parameters like filter stages
    UINT16,
    ULONG, // Added for unsigned long parameters like printIntervalMs
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
        MSP_API // Machine-readable MSP API mode.
    };

    FlightController *_fc;                              // Pointer to the FlightController instance
    OperatingMode _currentMode = OperatingMode::FLIGHT; // Current operating mode
    unsigned long _lastSerialLogTime = 0;               // Timestamp of the last serial log output
    bool _isSendingSettings = false;                    // Flag to prevent live_data stream during settings dump
    MspParser _mspParser;                               // MSP Parser instance

    static const Setting settingsRegistry[];
    static const int numSettings;

    static CommunicationManager *_instance; // Static instance to allow MSP callback to access members

    static constexpr float DEFAULT_SCALE_FACTOR = 1.0f;



    // Helper functions for string conversion
    String _getReceiverProtocolString(ReceiverProtocol protocol) const;
    String _getImuProtocolString(ImuProtocol protocol) const;
    String _getLpfBandwidthString(LpfBandwidth bandwidth) const;
    String _getFlightControlInputString(FlightControlInput input) const;
    String _getDShotModeString(dshot_mode_t mode) const;
    String _getImuRotationString(ImuRotation rotation) const;
    String _getBoolString(bool value) const;
    String _getUint8String(uint8_t value) const;
    String _getULongString(unsigned long value) const;
    String _payloadToString(const uint8_t* payload, uint16_t size) const;

    static constexpr int RX_MAP_PREFIX_LENGTH = 7; // Length of "rx.map."

    // Helper functions for parsing and validating setting values
    SetResult _parseAndValidateFloat(const String &valueStr, float &outValue, float scaleFactor, String &expectedValue) const;
    SetResult _parseAndValidateInt(const String &valueStr, int &outValue, String &expectedValue) const;
    SetResult _parseAndValidateUint16(const String &valueStr, uint16_t &outValue, String &expectedValue) const;
    SetResult _parseAndValidateULong(const String &valueStr, unsigned long &outValue, String &expectedValue) const;
    SetResult _parseAndValidateReceiverProtocol(const String &valueStr, ReceiverProtocol &outValue, String &expectedValue) const;
    SetResult _parseAndValidateImuProtocol(const String &valueStr, ImuProtocol &outValue, String &expectedValue) const;
    SetResult _parseAndValidateLpfBandwidth(const String &valueStr, LpfBandwidth &outValue, String &expectedValue) const;
    SetResult _parseAndValidateImuRotation(const String &valueStr, ImuRotation &outValue, String &expectedValue) const;
    SetResult _parseAndValidateDShotMode(const String &valueStr, dshot_mode_t &outValue, String &expectedValue) const;
    SetResult _parseAndValidateBool(const String &valueStr, bool &outValue, String &expectedValue) const;
    SetResult _parseAndValidateRxChannelMap(const String &param, const String &valueStr, int &outValue, String &expectedValue) const;

    // Private helper methods for command processing
    void _handleSerialInput();
    void _executeCommand(String command, bool isApiMode);
    void _handleSettingsCommand(String commandName, String commandArgs, bool isApiMode);
    void _handleSystemCommand(String commandName, bool isApiMode);
    void _handleUtilityCommand(String commandName, bool isApiMode);
    void _handleDumpCommand();
    void _printCliHelp();

    void _handleStatusCommand() const;
    void _handleVersionCommand() const;

    void _handleFlightModeInput(const String &input);
    void _handleMspApiInput();

    static void _onMspMessageReceived(const MspMessage &message, const char *prefix);
    void _handleMspCommand(const MspMessage &message);
    void _sendMspResponse(uint16_t command, const uint8_t *payload, uint16_t payloadSize);
    uint16_t _serializeSettingValueToMspPayload(const Setting *setting, uint8_t *buffer) const;
    SetResult _deserializeMspPayloadToSettingValue(const uint8_t *payload, uint16_t payloadSize, Setting *setting);
    uint16_t _serializeRxChannelMapToMspPayload(uint8_t *buffer) const;
    SetResult _deserializeMspPayloadToRxChannelMap(const uint8_t *payload, uint16_t payloadSize);
    uint16_t _serializeStatusToMspPayload(uint8_t *buffer) const;
    uint16_t _serializeVersionToMspPayload(uint8_t *buffer) const;
    uint16_t _serializeFlightStatusToMspPayload(uint8_t *buffer) const;
};

#endif // COMMUNICATION_MANAGER_H
