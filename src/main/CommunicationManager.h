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
#include <map>
#include <functional>

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

    FlightController *_flightController;                         // Pointer to the FlightController instance
    OperatingMode _currentOperatingMode = OperatingMode::FLIGHT; // Current operating mode
    unsigned long _lastTelemetrySendTimeUs = 0;                  // Timestamp of the last serial log output

    bool _isTelemetryStreamingEnabled = false; // Flag to control live data streaming
    MspParser _mspMessageParser;               // MSP Parser instance

    static const Setting settingsRegistry[];
    static const int numSettings;

    static CommunicationManager *_communicationManagerInstance; // Static instance to allow MSP callback to access members

    static constexpr float DEFAULT_SCALE_FACTOR = 1.0f;

    static constexpr uint16_t MSP_PAYLOAD_SIZE_STATUS = 1;
    static constexpr uint16_t MSP_MAX_PAYLOAD_SIZE_SETTINGS = 128;
    static constexpr uint16_t MSP_MAX_PAYLOAD_SIZE_STATUS_VERSION = 64;
    static constexpr uint16_t MSP_MAX_PAYLOAD_SIZE_FLIGHT_STATUS = 128;
    static constexpr uint8_t VERSION_STRING_BUFFER_SIZE = 16;
    static constexpr int PID_DISPLAY_SCALE_FACTOR = 10;
    static constexpr const char *RX_MAP_PREFIX = "rx.map.";

    // Helper functions for string conversion
    String _getReceiverProtocolName(ReceiverProtocol protocol) const;
    String _getImuProtocolName(ImuProtocol protocol) const;
    String _getLpfBandwidthName(LpfBandwidth bandwidth) const;
    String _getFlightControlInputName(FlightControlInput input) const;
    String _getDShotModeName(dshot_mode_t mode) const;
    String _getImuRotationName(ImuRotation rotation) const;
    String _getBoolName(bool value) const;
    String _getUint8Name(uint8_t value) const;
    String _getULongName(unsigned long value) const;
    String _convertPayloadToString(const uint8_t *payload, uint16_t size) const;

    static constexpr int RX_MAP_PREFIX_LENGTH = 7; // Length of "rx.map."

    // Helper functions for parsing and validating setting values
    bool _isStringNumericZero(const String &str) const;
    SetResult _parseAndValidateFloatSetting(const String &valueStr, float &outValue, float scaleFactor, String &expectedValue) const;
    SetResult _parseAndValidateIntSetting(const String &valueStr, int &outValue, String &expectedValue) const;
    SetResult _parseAndValidateUint16Setting(const String &valueStr, uint16_t &outValue, String &expectedValue) const;
    SetResult _parseAndValidateUint8Setting(const String &valueStr, uint8_t &outValue, String &expectedValue) const;
    SetResult _parseAndValidateULongSetting(const String &valueStr, unsigned long &outValue, String &expectedValue) const;
    SetResult _parseAndValidateReceiverProtocolSetting(const String &valueStr, ReceiverProtocol &outValue, String &expectedValue) const;
    SetResult _parseAndValidateImuProtocolSetting(const String &valueStr, ImuProtocol &outValue, String &expectedValue) const;
    SetResult _parseAndValidateLpfBandwidthSetting(const String &valueStr, LpfBandwidth &outValue, String &expectedValue) const;
    SetResult _parseAndValidateImuRotationSetting(const String &valueStr, ImuRotation &outValue, String &expectedValue) const;
    SetResult _parseAndValidateDShotModeSetting(const String &valueStr, dshot_mode_t &outValue, String &expectedValue) const;
    SetResult _parseAndValidateBoolSetting(const String &valueStr, bool &outValue, String &expectedValue) const;
    SetResult _parseAndValidateReceiverChannelMapSetting(const String &param, const String &valueStr, int &outValue, String &expectedValue) const;

    // Private helper methods for command processing
    void _processSerialInput();
    void _executeCliCommand(String command, bool isApiMode);
    void _processSystemCliCommand(String commandName, bool isApiMode);
    void _processUtilityCliCommand(String commandName, bool isApiMode);
    void _processGetSetCliCommand(String commandName, String commandArgs, bool isApiMode);
    void _processGetSettingCliCommand(String settingName, bool isApiMode);
    void _processSetSettingCliCommand(String commandArgs, bool isApiMode);
    void _displayAllSettingsCliCommand();
    void _displayCliHelp();

    // CLI command map
    std::map<String, std::function<void(String, bool)>> _cliCommandMap;
    void _initializeCliCommandMap();

    void _displaySystemStatusCliCommand() const;
    void _displayFirmwareVersionCliCommand() const;

    void _processFlightModeInput(const String &input);
    void _processMspApiInput();

    static void _onMspMessageReceivedCallback(const MspMessage &message, const char *prefix);
    void _processMspCommand(const MspMessage &message);
    void _sendMspMessageResponse(uint16_t command, const uint8_t *payload, uint16_t payloadSize);

    uint16_t _serializeSettingToMspPayload(const Setting *setting, uint8_t *buffer) const;
    uint16_t _deserializeMspPayloadToSetting(const uint8_t *payload, uint16_t payloadSize, Setting *setting, uint16_t offset);
    uint16_t _serializeReceiverChannelMapToMspPayload(uint8_t *buffer) const;
    SetResult _deserializeMspPayloadToReceiverChannelMap(const uint8_t *payload, uint16_t payloadSize);
    uint16_t _serializeFlightStatusToMspPayload(uint8_t *buffer) const;
    uint16_t _serializeFirmwareVersionToMspPayload(uint8_t *buffer) const;
    uint16_t _serializeLiveFlightDataToMspPayload(uint8_t *buffer) const;
};

#endif // COMMUNICATION_MANAGER_H
