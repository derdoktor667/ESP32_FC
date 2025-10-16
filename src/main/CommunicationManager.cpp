// CommunicationManager.cpp
//
// This file implements the CommunicationManager class, which handles all serial
// communication (CLI/API) and logging for the ESP32 Flight Controller. It
// manages different operating modes (FLIGHT, CLI, API) and processes incoming
// commands and outgoing flight status data.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "CommunicationManager.h"
#include "flight_controller.h"
#include "../config/config.h"
#include "../config/settings.h"
#include "../config/FlightState.h"
#include <Arduino.h>

// --- Refactoring: Settings Registry ---

enum class SettingType {
    FLOAT,
    UINT16,
    DSHOT_MODE,
    RECEIVER_PROTOCOL,
    IMU_PROTOCOL,
    LPF_BANDWIDTH,
    BOOL
};

struct Setting {
    const char* name;
    SettingType type;
    void* value;
    const float scaleFactor;
};

// --- Helper Functions for String Conversion ---
static String getReceiverProtocolString(ReceiverProtocol protocol) {
    if (protocol == PROTOCOL_IBUS) return "IBUS";
    if (protocol == PROTOCOL_PPM) return "PPM";
    return "UNKNOWN";
}

static String getImuProtocolString(ImuProtocol protocol) {
    if (protocol == IMU_MPU6050) return "MPU6050";
    return "UNKNOWN";
}

static String getLpfBandwidthString(LpfBandwidth bandwidth) {
    switch (bandwidth) {
        case LPF_256HZ_N_0MS: return "LPF_256HZ_N_0MS";
        case LPF_188HZ_N_2MS: return "LPF_188HZ_N_2MS";
        case LPF_98HZ_N_3MS: return "LPF_98HZ_N_3MS";
        case LPF_42HZ_N_5MS: return "LPF_42HZ_N_5MS";
        case LPF_20HZ_N_10MS: return "LPF_20HZ_N_10MS";
        case LPF_10HZ_N_13MS: return "LPF_10HZ_N_13MS";
        case LPF_5HZ_N_18MS: return "LPF_5HZ_N_18MS";
        default: return "UNKNOWN";
    }
}

static String getFlightControlInputString(FlightControlInput input) {
    switch (input) {
        case THROTTLE: return "THROTTLE";
        case ROLL: return "ROLL";
        case PITCH: return "PITCH";
        case YAW: return "YAW";
        case ARM_SWITCH: return "ARM_SWITCH";
        case FAILSAFE_SWITCH: return "FAILSAFE_SWITCH";
        case FLIGHT_MODE_SWITCH: return "FLIGHT_MODE_SWITCH";
        default: return "UNKNOWN";
    }
}

static String getDShotModeString(dshot_mode_t mode) {
    if (mode == DSHOT_OFF) return "DSHOT_OFF";
    if (mode == DSHOT150) return "DSHOT150";
    if (mode == DSHOT300) return "DSHOT300";
    if (mode == DSHOT600) return "DSHOT600";
    if (mode == DSHOT1200) return "DSHOT1200";
    return "UNKNOWN";
}

static String getBoolString(bool value) {
    return value ? "true" : "false";
}

static const Setting settingsRegistry[] = {
    { "pid.roll.kp", SettingType::UINT16, &settings.pidRoll.kp, PID_SCALE_FACTOR },
    { "pid.roll.ki", SettingType::UINT16, &settings.pidRoll.ki, PID_SCALE_FACTOR },
    { "pid.roll.kd", SettingType::UINT16, &settings.pidRoll.kd, PID_SCALE_FACTOR },
    { "pid.pitch.kp", SettingType::UINT16, &settings.pidPitch.kp, PID_SCALE_FACTOR },
    { "pid.pitch.ki", SettingType::UINT16, &settings.pidPitch.ki, PID_SCALE_FACTOR },
    { "pid.pitch.kd", SettingType::UINT16, &settings.pidPitch.kd, PID_SCALE_FACTOR },
    { "pid.yaw.kp", SettingType::UINT16, &settings.pidYaw.kp, PID_SCALE_FACTOR },
    { "pid.yaw.ki", SettingType::UINT16, &settings.pidYaw.ki, PID_SCALE_FACTOR },
    { "pid.yaw.kd", SettingType::UINT16, &settings.pidYaw.kd, PID_SCALE_FACTOR },
    { "pid.integral_limit", SettingType::FLOAT, &settings.pidIntegralLimit, 1.0f },
    { "rates.angle", SettingType::FLOAT, &settings.rates.maxAngleRollPitch, 1.0f },
    { "rates.yaw", SettingType::FLOAT, &settings.rates.maxRateYaw, 1.0f },
    { "rates.acro", SettingType::FLOAT, &settings.rates.maxRateRollPitch, 1.0f },
    { "madgwick.sample_freq", SettingType::FLOAT, &settings.filter.madgwickSampleFreq, 1.0f },
    { "madgwick.beta", SettingType::FLOAT, &settings.filter.madgwickBeta, 1.0f },
    { "rx.min", SettingType::UINT16, &settings.receiver.ibusMinValue, 1.0f },
    { "rx.max", SettingType::UINT16, &settings.receiver.ibusMaxValue, 1.0f },
    { "rx.arming_threshold", SettingType::UINT16, &settings.receiver.armingThreshold, 1.0f },
    { "rx.failsafe_threshold", SettingType::UINT16, &settings.receiver.failsafeThreshold, 1.0f },
    { "rx.protocol", SettingType::RECEIVER_PROTOCOL, &settings.receiverProtocol, 1.0f },
    { "imu.protocol", SettingType::IMU_PROTOCOL, &settings.imuProtocol, 1.0f },
    { "imu.lpf", SettingType::LPF_BANDWIDTH, &settings.imuLpfBandwidth, 1.0f },
    { "motor.idle_speed", SettingType::FLOAT, &settings.motorIdleSpeedPercent, 1.0f },
    { "motor.dshot_mode", SettingType::DSHOT_MODE, &settings.dshotMode, 1.0f },
    { "enforce_loop_time", SettingType::BOOL, &settings.enforceLoopTime, 1.0f },
};
const int numSettings = sizeof(settingsRegistry) / sizeof(Setting);

// --- Helper Functions for Parsing and Validation ---

CommunicationManager::SetResult CommunicationManager::_parseAndValidateFloat(const String& valueStr, float& outValue, float scaleFactor, String& expectedValue) {
    float val = valueStr.toFloat();
    if (valueStr.length() > 0 && val == 0.0f && valueStr != "0" && valueStr != "0.0") {
        expectedValue = "float";
        return SetResult::INVALID_VALUE;
    }
    outValue = val * scaleFactor;
    return SetResult::SUCCESS;
}

CommunicationManager::SetResult CommunicationManager::_parseAndValidateUint16(const String& valueStr, uint16_t& outValue, String& expectedValue) {
    long val = valueStr.toInt();
    if (valueStr.length() > 0 && val == 0 && valueStr != "0") {
        expectedValue = "integer";
        return SetResult::INVALID_VALUE;
    } else if (val < 0 || val > 65535) { // uint16_t range
        expectedValue = "0-65535";
        return SetResult::OUT_OF_RANGE;
    }
    outValue = (uint16_t)val;
    return SetResult::SUCCESS;
}

CommunicationManager::SetResult CommunicationManager::_parseAndValidateReceiverProtocol(const String& valueStr, ReceiverProtocol& outValue, String& expectedValue) {
    if (valueStr.equalsIgnoreCase("IBUS")) outValue = PROTOCOL_IBUS;
    else if (valueStr.equalsIgnoreCase("PPM")) outValue = PROTOCOL_PPM;
    else { expectedValue = "IBUS, PPM"; return SetResult::INVALID_VALUE; }
    return SetResult::SUCCESS;
}

CommunicationManager::SetResult CommunicationManager::_parseAndValidateImuProtocol(const String& valueStr, ImuProtocol& outValue, String& expectedValue) {
    if (valueStr.equalsIgnoreCase("MPU6050")) outValue = IMU_MPU6050;
    else { expectedValue = "MPU6050"; return SetResult::INVALID_VALUE; }
    return SetResult::SUCCESS;
}

CommunicationManager::SetResult CommunicationManager::_parseAndValidateLpfBandwidth(const String& valueStr, LpfBandwidth& outValue, String& expectedValue) {
    if (valueStr.equalsIgnoreCase("LPF_256HZ_N_0MS")) outValue = LPF_256HZ_N_0MS;
    else if (valueStr.equalsIgnoreCase("LPF_188HZ_N_2MS")) outValue = LPF_188HZ_N_2MS;
    else if (valueStr.equalsIgnoreCase("LPF_98HZ_N_3MS")) outValue = LPF_98HZ_N_3MS;
    else if (valueStr.equalsIgnoreCase("LPF_42HZ_N_5MS")) outValue = LPF_42HZ_N_5MS;
    else if (valueStr.equalsIgnoreCase("LPF_20HZ_N_10MS")) outValue = LPF_20HZ_N_10MS;
    else if (valueStr.equalsIgnoreCase("LPF_10HZ_N_13MS")) outValue = LPF_10HZ_N_13MS;
    else if (valueStr.equalsIgnoreCase("LPF_5HZ_N_18MS")) outValue = LPF_5HZ_N_18MS;
    else { expectedValue = "LPF_256HZ_N_0MS, LPF_188HZ_N_2MS, LPF_98HZ_N_3MS, LPF_42HZ_N_5MS, LPF_20HZ_N_10MS, LPF_10HZ_N_13MS, LPF_5HZ_N_18MS"; return SetResult::INVALID_VALUE; }
    return SetResult::SUCCESS;
}

CommunicationManager::SetResult CommunicationManager::_parseAndValidateDShotMode(const String& valueStr, dshot_mode_t& outValue, String& expectedValue) {
    if (valueStr.equalsIgnoreCase("DSHOT_OFF")) outValue = DSHOT_OFF;
    else if (valueStr.equalsIgnoreCase("DSHOT150")) outValue = DSHOT150;
    else if (valueStr.equalsIgnoreCase("DSHOT300")) outValue = DSHOT300;
    else if (valueStr.equalsIgnoreCase("DSHOT600")) outValue = DSHOT600;
    else if (valueStr.equalsIgnoreCase("DSHOT1200")) outValue = DSHOT1200;
    else { expectedValue = "DSHOT_OFF, DSHOT150, DSHOT300, DSHOT600, DSHOT1200"; return SetResult::INVALID_VALUE; }
    return SetResult::SUCCESS;
}

CommunicationManager::SetResult CommunicationManager::_parseAndValidateBool(const String& valueStr, bool& outValue, String& expectedValue) {
    if (valueStr.equalsIgnoreCase("true")) outValue = true;
    else if (valueStr.equalsIgnoreCase("false")) outValue = false;
    else { expectedValue = "true, false"; return SetResult::INVALID_VALUE; }
    return SetResult::SUCCESS;
}

CommunicationManager::SetResult CommunicationManager::_parseAndValidateRxChannelMap(const String& param, const String& valueStr, int& outValue, String& expectedValue) {
    String inputName = param.substring(RX_MAP_PREFIX_LENGTH);
    int channelValue = valueStr.toInt();

    if (valueStr.length() > 0 && channelValue == 0 && valueStr != "0") {
        expectedValue = "integer";
        return SetResult::INVALID_VALUE;
    } else if (channelValue < 0 || channelValue >= RECEIVER_CHANNEL_COUNT) {
        expectedValue = "0-" + String(RECEIVER_CHANNEL_COUNT - 1);
        return SetResult::OUT_OF_RANGE;
    } else {
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
            if (inputName.equalsIgnoreCase(getFlightControlInputString((FlightControlInput)i))) {
                outValue = channelValue;
                return SetResult::SUCCESS;
            }
        }
    }
    return SetResult::UNKNOWN_PARAMETER;
}

// --- Command Helpers for Get/Set ---

// Helper function to print a GET response to serial, formatted as JSON for API mode or plain text for CLI mode.
void _printGetResponse(const String& param, const String& value, bool isApiMode, bool isString) {
    if (isApiMode) {
        Serial.print("{\"get\":{\"");
        Serial.print(param);
        Serial.print("\":");
        if (isString) Serial.print("\"");
        Serial.print(value);
        if (isString) Serial.print("\"");
        Serial.println("}}");
    } else {
        Serial.println(value);
    }
}

// Helper function to print a SET response to serial, formatted as JSON for API mode or plain text for CLI mode.
// Provides detailed error messages based on the SetResult enum.
void _printSetResponse(const String& param, const String& value, CommunicationManager::SetResult result, bool isApiMode, bool isString, const String& expected) {
    if (isApiMode) {
        Serial.print("{\"set\":{\"");
        Serial.print(param);
        Serial.print("\":");
        if (isString) Serial.print("\"");
        Serial.print(value);
        if (isString) Serial.print("\"");
        Serial.print(",\"status\":\"");
        switch (result) {
            case CommunicationManager::SetResult::SUCCESS: Serial.print("success"); break;
            case CommunicationManager::SetResult::INVALID_FORMAT: Serial.print("error"); Serial.print(",\"message\":\"Invalid 'set' command format. Use: set <parameter> <value>\""); break;
            case CommunicationManager::SetResult::UNKNOWN_PARAMETER: Serial.print("error"); Serial.print(",\"message\":\"Unknown parameter\""); break;
            case CommunicationManager::SetResult::INVALID_VALUE: Serial.print("error"); Serial.print(",\"message\":\"Invalid value. Expected: "); Serial.print(expected); Serial.print("\""); break;
            case CommunicationManager::SetResult::OUT_OF_RANGE: Serial.print("error"); Serial.print(",\"message\":\"Value out of range. Expected: "); Serial.print(expected); Serial.print("\""); break;
        }
        Serial.println("}}");
    } else {
        switch (result) {
            case CommunicationManager::SetResult::SUCCESS:
                Serial.print("Set "); Serial.print(param); Serial.print(" to "); Serial.println(value);
                break;
            case CommunicationManager::SetResult::INVALID_FORMAT:
                Serial.println("Invalid 'set' format. Use: set <parameter> <value>");
                break;
            case CommunicationManager::SetResult::UNKNOWN_PARAMETER:
                Serial.print("Unknown parameter: "); Serial.println(param);
                break;
            case CommunicationManager::SetResult::INVALID_VALUE:
                Serial.print("Invalid value '" ); Serial.print(value); Serial.print("' for parameter '" ); Serial.print(param); Serial.print("'. Expected: "); Serial.println(expected);
                break;
            case CommunicationManager::SetResult::OUT_OF_RANGE:
                Serial.print("Value '" ); Serial.print(value); Serial.print("' for parameter '" ); Serial.print(param); Serial.print("' is out of range. Expected: "); Serial.println(expected);
                break;
        }
    }
}

// Helper function to print live flight status data to serial, formatted as JSON for API mode.
void _printFlightStatus(const FlightState &state) {
    if (isnan(state.attitude.roll) || isnan(state.attitude.pitch) || isnan(state.attitude.yaw)) {
        Serial.println("{\"error\":\"Attitude data is NaN. Check IMU connection.\"}");
        return;
    }
    Serial.print("{\"live_data\":{\"attitude\":{\"roll\":");
    Serial.print(state.attitude.roll, 2);
    Serial.print(",\"pitch\":");
    Serial.print(state.attitude.pitch, 2);
    Serial.print(",\"yaw\":");
    Serial.print(state.attitude.yaw, 2);
    Serial.print("},\"status\":{\"armed\":");
    Serial.print(state.isArmed ? "true" : "false");
    Serial.print(",\"failsafe\":");
    Serial.print(state.isFailsafeActive ? "true" : "false");
    Serial.print(",\"mode\":\"");
    switch (state.currentFlightMode) {
        case ACRO_MODE: Serial.print("ACRO"); break;
        case ANGLE_MODE: Serial.print("ANGLE"); break;
        default: Serial.print("UNKNOWN"); break;
    }
    Serial.print("\"},\"motor_output\":[");
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print(state.motorOutputs[i]);
        if (i < NUM_MOTORS - 1) Serial.print(",");
    }
    Serial.print("],\"receiver_channels\":[");
    for (int i = 0; i < RECEIVER_CHANNEL_COUNT; i++) {
        Serial.print(state.receiverChannels[i]);
        if (i < RECEIVER_CHANNEL_COUNT - 1) Serial.print(",");
    }
    Serial.println("]}}");
}

// --- Public Methods ---

CommunicationManager::CommunicationManager(FlightController* fc) : _fc(fc) {}

void CommunicationManager::begin() {}

void CommunicationManager::update(const FlightState &state) {
    _handleSerialInput();
    if (_currentMode == OperatingMode::API && !_isSendingSettings && settings.enableLogging && millis() - _lastSerialLogTime >= settings.printIntervalMs) {
        _printFlightStatus(state);
        _lastSerialLogTime = millis();
    }
}

// --- Private Methods: Main Logic ---

void CommunicationManager::_handleSerialInput() {
    if (Serial.available() == 0) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    switch (_currentMode) {
        case OperatingMode::FLIGHT:
            _handleFlightModeInput(input);
            break;
        case OperatingMode::CLI: {
            String commandName = (input.indexOf(' ') != -1) ? input.substring(0, input.indexOf(' ')) : input;
            if (commandName.equalsIgnoreCase("exit")) {
                _currentMode = OperatingMode::FLIGHT;
                Serial.println("--- CLI  Deactivated ---");
                return;
            }
            _executeCommand(input, false);
            if (!commandName.equalsIgnoreCase("save") && !commandName.equalsIgnoreCase("reboot") && !commandName.equalsIgnoreCase("reset")) {
                Serial.print("ESP32_FC > ");
            }
            break;
        }
        case OperatingMode::API:
            _executeCommand(input, true);
            break;
    }
}

void CommunicationManager::_handleFlightModeInput(const String& input) {
    if (input.equalsIgnoreCase("cli")) {
        _currentMode = OperatingMode::CLI;
        settings.enableLogging = false;
        Serial.println("--- CLI  Activated ---");
        Serial.print("ESP32_FC > ");
    } else if (input.equalsIgnoreCase("api")) {
        _currentMode = OperatingMode::API;
        settings.enableLogging = true;
        Serial.println("{\"status\":\"api_mode_activated\"}");
    }
}

void CommunicationManager::_executeCommand(String command, bool isApiMode) {
    command.toLowerCase();
    String commandName = "";
    String commandArgs = "";
    int firstSpace = command.indexOf(' ');
    if (firstSpace != -1) {
        commandName = command.substring(0, firstSpace);
        commandArgs = command.substring(firstSpace + 1);
    } else {
        commandName = command;
    }

    if (commandName.equals("get")) _handleGetCommand(commandArgs, isApiMode);
    else if (commandName.equals("set")) _handleSetCommand(commandArgs, isApiMode);
    else if (commandName.equals("dump") && !isApiMode) _handleDumpCommand();
    else if (commandName.equals("get_settings") && isApiMode) _handleDumpJsonCommand();
    else if (commandName.equals("save")) {
        if (!isApiMode) Serial.println("INFO: Settings saved. Rebooting...");
        saveSettings();
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    } else if (commandName.equals("reset")) {
        if (!isApiMode) Serial.println("INFO: All settings have been reset to their default values and saved.");
        settings = FlightControllerSettings();
        saveSettings();
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    } else if (commandName.equals("reboot")) {
        if (!isApiMode) Serial.println("Rebooting...");
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    } else if (commandName.equals("calibrate_imu")) {
        if (!isApiMode) Serial.println("INFO: IMU calibration requested.");
        _fc->requestImuCalibration();
    } else if (commandName.equals("status") && !isApiMode) {
        _handleStatusCommand();
    } else if (commandName.equals("version")) {
        _handleVersionCommand();
    } else if (commandName.equals("help") && !isApiMode) {
        _printCliHelp();
    } else {
        if (isApiMode) Serial.println("{\"error\":\"Unknown command\"}");
        else { Serial.print("Unknown command: "); Serial.println(commandName); }
    }
}

// --- Refactored Command Implementations ---

void CommunicationManager::_printCliHelp() {
    Serial.println("Available Commands:");
    Serial.println("  get <parameter>    - Get the current value of a setting.");
    Serial.println("  set <parameter> <value> - Set a new value for a setting.");
    Serial.println("  dump             - Display all current settings and their values.");
    Serial.println("  get_settings       - (API only) Get all settings as JSON.");
    Serial.println("  save             - Save current settings to flash and reboot.");
    Serial.println("  reset            - Reset all settings to default and reboot.");
    Serial.println("  reboot           - Reboot the ESP32.");
    Serial.println("  calibrate_imu    - Request IMU calibration.");
    Serial.println("  status           - Display system status and metrics.");
    Serial.println("  version          - Display firmware version."); // New command
    Serial.println("  help             - Display this help message.");
    Serial.println("  exit             - Exit CLI mode.");
    Serial.println("");
    Serial.println("Available Settings (parameter names for 'get' and 'set'):");

    // Group settings by category for better readability
    Serial.println("  PID Gains:");
    Serial.println("    pid.roll.kp, pid.roll.ki, pid.roll.kd");
    Serial.println("    pid.pitch.kp, pid.pitch.ki, pid.pitch.kd");
    Serial.println("    pid.yaw.kp, pid.yaw.ki, pid.yaw.kd");
    Serial.println("    pid.integral_limit");

    Serial.println("  Rates & Angles:");
    Serial.println("    rates.angle (Max Roll/Pitch Angle in Angle Mode)");
    Serial.println("    rates.yaw (Max Yaw Rate)");
    Serial.println("    rates.acro (Max Roll/Pitch Rate in Acro Mode)");

    Serial.println("  Madgwick Filter:");
    Serial.println("    madgwick.sample_freq");
    Serial.println("    madgwick.beta");

    Serial.println("  Receiver:");
    Serial.println("    rx.min, rx.max (Min/Max raw receiver values)");
    Serial.println("    rx.arming_threshold, rx.failsafe_threshold");
    Serial.println("    rx.protocol (IBUS, PPM)");
    Serial.println("    rx.map.<input> <channel> (e.g., rx.map.throttle 1)");

    Serial.println("  IMU:");
    Serial.println("    imu.protocol (MPU6050)");
    Serial.println("    imu.lpf (LPF_256HZ_N_0MS, LPF_188HZ_N_2MS, LPF_98HZ_N_3MS, LPF_42HZ_N_5MS, LPF_20HZ_N_10MS, LPF_10HZ_N_13MS, LPF_5HZ_N_18MS)");

    Serial.println("  Motor:");
    Serial.println("    motor.idle_speed (Percent)");
    Serial.println("    motor.dshot_mode (DSHOT_OFF, DSHOT150, DSHOT300, DSHOT600, DSHOT1200)");

    Serial.println("  Logging:");
    Serial.println("    printIntervalMs (Interval for serial logging in ms)");
    Serial.println("    enableLogging (true/false)");
    Serial.println("    enforce_loop_time (true/false) - Enforce target loop time with delayMicroseconds");
}

void CommunicationManager::_handleDumpCommand() {
    Serial.println("--- Current Flight Controller Settings ---");
    for (int i = 0; i < numSettings; ++i) {
        const Setting& s = settingsRegistry[i];
        Serial.print(s.name);
        Serial.print(": ");
        switch (s.type) {
            case SettingType::FLOAT: Serial.println(*(float*)s.value / s.scaleFactor, 4); break;
            case SettingType::UINT16: Serial.println(*(uint16_t*)s.value); break;
            case SettingType::RECEIVER_PROTOCOL: Serial.println(getReceiverProtocolString(*(ReceiverProtocol*)s.value)); break;
            case SettingType::IMU_PROTOCOL: Serial.println(getImuProtocolString(*(ImuProtocol*)s.value)); break;
            case SettingType::LPF_BANDWIDTH: Serial.println(getLpfBandwidthString(*(LpfBandwidth*)s.value)); break;
            case SettingType::BOOL: Serial.println(getBoolString(*(bool*)s.value)); break;
            case SettingType::DSHOT_MODE: Serial.println(getDShotModeString(*(dshot_mode_t*)s.value)); break;
        }
    }
    Serial.println("\n--- Receiver Channel Mapping ---");
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
        Serial.print("  ");
        Serial.print(getFlightControlInputString((FlightControlInput)i));
        Serial.print(": ");
        Serial.println(settings.channelMapping.channel[i]);
    }
    Serial.println("----------------------------------------");
}

void CommunicationManager::_handleDumpJsonCommand() {
    _isSendingSettings = true;
    Serial.print("{\"settings\":{");
    delay(1);
    for (int i = 0; i < numSettings; ++i) {
        const Setting& s = settingsRegistry[i];
        if (i > 0) {
            Serial.print(",");
            delay(1);
        }
        Serial.print("\"");
        Serial.print(s.name);
        Serial.print("\":");
        delay(1);
        switch (s.type) {
            case SettingType::FLOAT:
                Serial.print(*(float*)s.value / s.scaleFactor, 4);
                break;
            case SettingType::UINT16:
                Serial.print(*(uint16_t*)s.value);
                break;
            case SettingType::RECEIVER_PROTOCOL:
                Serial.print("\"");
                Serial.print(getReceiverProtocolString(*(ReceiverProtocol*)s.value));
                Serial.print("\"");
                break;
            case SettingType::IMU_PROTOCOL:
                Serial.print("\"");
                Serial.print(getImuProtocolString(*(ImuProtocol*)s.value));
                Serial.print("\"");
                break;
            case SettingType::LPF_BANDWIDTH:
                Serial.print("\"");
                Serial.print(getLpfBandwidthString(*(LpfBandwidth*)s.value));
                Serial.print("\"");
                break;
            case SettingType::BOOL:
                Serial.print("\"");
                Serial.print(getBoolString(*(bool*)s.value));
                Serial.print("\"");
                break;
            case SettingType::DSHOT_MODE:
                Serial.print("\"");
                Serial.print(getDShotModeString(*(dshot_mode_t*)s.value));
                Serial.print("\"");
                break;
        }
        delay(1);
    }
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
        String key = "rx.map.";
        String inputName = getFlightControlInputString((FlightControlInput)i);
        inputName.toLowerCase();
        key += inputName;
        Serial.print(",\"");
        Serial.print(key);
        Serial.print("\":");
        Serial.print(settings.channelMapping.channel[i]);
        delay(1);
    }
    Serial.println("}}");
    _isSendingSettings = false;
}

void CommunicationManager::_handleGetCommand(String args, bool isApiMode) {
    for (int i = 0; i < numSettings; ++i) {
        const Setting& s = settingsRegistry[i];
        if (args.equalsIgnoreCase(s.name)) {
            String valueStr;
            bool isString = false;
            switch (s.type) {
                case SettingType::FLOAT: valueStr = String(*(float*)s.value / s.scaleFactor, 4); break;
                case SettingType::UINT16: valueStr = String(*(uint16_t*)s.value); break;
                case SettingType::RECEIVER_PROTOCOL: valueStr = getReceiverProtocolString(*(ReceiverProtocol*)s.value); isString = true; break;
                case SettingType::IMU_PROTOCOL: valueStr = getImuProtocolString(*(ImuProtocol*)s.value); isString = true; break;
                case SettingType::LPF_BANDWIDTH: valueStr = getLpfBandwidthString(*(LpfBandwidth*)s.value); isString = true; break;
                case SettingType::BOOL: valueStr = getBoolString(*(bool*)s.value); isString = true; break;
                case SettingType::DSHOT_MODE: valueStr = getDShotModeString(*(dshot_mode_t*)s.value); isString = true; break;
            }
            _printGetResponse(args, valueStr, isApiMode, isString);
            return;
        }
    }
    if (isApiMode) { Serial.println("{\"error\":\"Unknown parameter for get\"}"); } 
    else { Serial.println("Unknown parameter for 'get'."); }
}

void CommunicationManager::_handleSetCommand(String args, bool isApiMode) {
    int lastSpace = args.lastIndexOf(' ');
    if (lastSpace == -1) {
        _printSetResponse("", "", CommunicationManager::SetResult::INVALID_FORMAT, isApiMode, false, "");
        return;
    }
    String param = args.substring(0, lastSpace);
    String valueStr = args.substring(lastSpace + 1);

    for (int i = 0; i < numSettings; ++i) {
        const Setting& s = settingsRegistry[i];
        if (param.equalsIgnoreCase(s.name)) {
            CommunicationManager::SetResult result = CommunicationManager::SetResult::SUCCESS;
            String expectedValue = "";

            switch (s.type) {
                case SettingType::FLOAT: {
                    float val;
                    result = _parseAndValidateFloat(valueStr, val, s.scaleFactor, expectedValue);
                    if (result == SetResult::SUCCESS) *(float*)s.value = val;
                    break;
                }
                case SettingType::UINT16: {
                    uint16_t val;
                    result = _parseAndValidateUint16(valueStr, val, expectedValue);
                    if (result == SetResult::SUCCESS) *(uint16_t*)s.value = val;
                    break;
                }
                case SettingType::RECEIVER_PROTOCOL: {
                    ReceiverProtocol val;
                    result = _parseAndValidateReceiverProtocol(valueStr, val, expectedValue);
                    if (result == SetResult::SUCCESS) *(ReceiverProtocol*)s.value = val;
                    break;
                }
                case SettingType::IMU_PROTOCOL: {
                    ImuProtocol val;
                    result = _parseAndValidateImuProtocol(valueStr, val, expectedValue);
                    if (result == SetResult::SUCCESS) *(ImuProtocol*)s.value = val;
                    break;
                }
                case SettingType::LPF_BANDWIDTH: {
                    LpfBandwidth val;
                    result = _parseAndValidateLpfBandwidth(valueStr, val, expectedValue);
                    if (result == SetResult::SUCCESS) *(LpfBandwidth*)s.value = val;
                    break;
                }
                case SettingType::DSHOT_MODE: {
                    dshot_mode_t val;
                    result = _parseAndValidateDShotMode(valueStr, val, expectedValue);
                    if (result == SetResult::SUCCESS) *(dshot_mode_t*)s.value = val;
                    break;
                }
            }
            _printSetResponse(param, valueStr, result, isApiMode, (s.type == SettingType::DSHOT_MODE || s.type == SettingType::RECEIVER_PROTOCOL || s.type == SettingType::IMU_PROTOCOL || s.type == SettingType::LPF_BANDWIDTH || s.type == SettingType::BOOL), expectedValue);
            return;
        }
    }

    // Handle rx.map settings
    if (param.startsWith("rx.map.")) {
        int channelValue;
        String expectedValue = "";
        CommunicationManager::SetResult result = _parseAndValidateRxChannelMap(param, valueStr, channelValue, expectedValue);

        if (result == SetResult::SUCCESS) {
            String inputName = param.substring(RX_MAP_PREFIX_LENGTH);
            for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
                if (inputName.equalsIgnoreCase(getFlightControlInputString((FlightControlInput)i))) {
                    settings.channelMapping.channel[i] = channelValue;
                    break;
                }
            }
        }
        _printSetResponse(param, valueStr, result, isApiMode, false, expectedValue);
        return;
    }

    _printSetResponse(param, valueStr, CommunicationManager::SetResult::UNKNOWN_PARAMETER, isApiMode, false, "");
}




void CommunicationManager::_handleStatusCommand() {
    Serial.println("--- System Status ---");
    Serial.print("Target Loop Time (us): "); Serial.println(TARGET_LOOP_TIME_US);
    Serial.print("Actual Loop Time (us): "); Serial.println(_fc->state.loopTimeUs);
    Serial.print("CPU Load (%): "); Serial.println(_fc->state.cpuLoad, 2);
    Serial.print("Battery Voltage (V): "); Serial.println(_fc->state.voltage, 2);
    Serial.print("Current Draw (A): "); Serial.println(_fc->state.current, 2);
    Serial.print("Free Heap (bytes): "); Serial.println(esp_get_free_heap_size());
    Serial.print("Min Free Heap (bytes): "); Serial.println(esp_get_minimum_free_heap_size());
    Serial.println("---------------------");
}

void CommunicationManager::_handleVersionCommand() {
    if (_currentMode == OperatingMode::API) {
        Serial.print("{\"version\":\"");
        Serial.print(FIRMWARE_VERSION);
        Serial.println("\"}");
    } else {
        Serial.print("Firmware Version: ");
        Serial.println(FIRMWARE_VERSION);
    }
}