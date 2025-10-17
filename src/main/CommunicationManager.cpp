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
#include <ArduinoJson.h>

// --- Refactoring: Settings Registry ---

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

static String getUint8String(uint8_t value)
{
    return String(value);
}

// --- Refactoring: Settings Registry ---

const CommunicationManager::Setting CommunicationManager::settingsRegistry[] = {
    { "pid.roll.kp", CommunicationManager::SettingType::UINT16, &settings.pidRoll.kp, PID_SCALE_FACTOR },
    { "pid.roll.ki", CommunicationManager::SettingType::UINT16, &settings.pidRoll.ki, PID_SCALE_FACTOR },
    { "pid.roll.kd", CommunicationManager::SettingType::UINT16, &settings.pidRoll.kd, PID_SCALE_FACTOR },
    { "pid.pitch.kp", CommunicationManager::SettingType::UINT16, &settings.pidPitch.kp, PID_SCALE_FACTOR },
    { "pid.pitch.ki", CommunicationManager::SettingType::UINT16, &settings.pidPitch.ki, PID_SCALE_FACTOR },
    { "pid.pitch.kd", CommunicationManager::SettingType::UINT16, &settings.pidPitch.kd, PID_SCALE_FACTOR },
    { "pid.yaw.kp", CommunicationManager::SettingType::UINT16, &settings.pidYaw.kp, PID_SCALE_FACTOR },
    { "pid.yaw.ki", CommunicationManager::SettingType::UINT16, &settings.pidYaw.ki, PID_SCALE_FACTOR },
    { "pid.yaw.kd", CommunicationManager::SettingType::UINT16, &settings.pidYaw.kd, PID_SCALE_FACTOR },
    { "pid.integral_limit", CommunicationManager::SettingType::FLOAT, &settings.pidIntegralLimit, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "rates.angle", CommunicationManager::SettingType::FLOAT, &settings.rates.maxAngleRollPitch, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "rates.yaw", CommunicationManager::SettingType::FLOAT, &settings.rates.maxRateYaw, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "rates.acro", CommunicationManager::SettingType::FLOAT, &settings.rates.maxRateRollPitch, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "filter.comp_tau", CommunicationManager::SettingType::FLOAT, &settings.filter.complementaryFilterTau, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "gyro.lpf_cutoff_freq", CommunicationManager::SettingType::FLOAT, &settings.filter.gyroLpfCutoffFreq, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "accel.lpf_cutoff_freq", CommunicationManager::SettingType::FLOAT, &settings.filter.accelLpfCutoffFreq, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "gyro.lpf_stages", CommunicationManager::SettingType::UINT8, &settings.filter.gyroLpfStages, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "accel.lpf_stages", CommunicationManager::SettingType::UINT8, &settings.filter.accelLpfStages, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "filter.sample_freq", CommunicationManager::SettingType::FLOAT, &settings.filter.filterSampleFreq, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "rx.min", CommunicationManager::SettingType::UINT16, &settings.receiver.ibusMinValue, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "rx.max", CommunicationManager::SettingType::UINT16, &settings.receiver.ibusMaxValue, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "rx.arming_threshold", CommunicationManager::SettingType::UINT16, &settings.receiver.armingThreshold, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "rx.failsafe_threshold", CommunicationManager::SettingType::UINT16, &settings.receiver.failsafeThreshold, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "rx.protocol", CommunicationManager::SettingType::ENUM_IBUS_PROTOCOL, &settings.receiverProtocol, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "imu.protocol", CommunicationManager::SettingType::ENUM_IMU_PROTOCOL, &settings.imuProtocol, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "imu.lpf", CommunicationManager::SettingType::ENUM_LPF_BANDWIDTH, &settings.imuLpfBandwidth, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "motor.idle_speed", CommunicationManager::SettingType::FLOAT, &settings.motorIdleSpeedPercent, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "motor.dshot_mode", CommunicationManager::SettingType::ENUM_DSHOT_MODE, &settings.dshotMode, CommunicationManager::DEFAULT_SCALE_FACTOR },
    { "enforce_loop_time", CommunicationManager::SettingType::BOOL, &settings.enforceLoopTime, CommunicationManager::DEFAULT_SCALE_FACTOR },
};
const int CommunicationManager::numSettings = sizeof(CommunicationManager::settingsRegistry) / sizeof(CommunicationManager::Setting);

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
    } else if (val < 0 || val > MAX_UINT16_VALUE) { // uint16_t range
        expectedValue = "0-" + String(MAX_UINT16_VALUE);
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
        StaticJsonDocument<CommunicationManager::JSON_DOC_MEDIUM_SIZE> doc;
        JsonObject get = doc.createNestedObject("get");
        if (isString) {
            get[param] = value;
        } else {
            get[param] = serialized(value);
        }
        serializeJson(doc, Serial);
        Serial.println();
    } else {
        Serial.println(value);
    }
}

// Helper function to print a SET response to serial, formatted as JSON for API mode or plain text for CLI mode.
// Provides detailed error messages based on the SetResult enum.
void _printSetResponse(const String& param, const String& value, CommunicationManager::SetResult result, bool isApiMode, bool isString, const String& expected) {
    if (isApiMode) {
        StaticJsonDocument<CommunicationManager::JSON_DOC_LARGE_SIZE> doc;
        JsonObject set = doc.createNestedObject("set");
        if (isString) {
            set[param] = value;
        } else {
            set[param] = serialized(value);
        }

        switch (result) {
            case CommunicationManager::SetResult::SUCCESS:
                set["status"] = "success";
                break;
            case CommunicationManager::SetResult::INVALID_FORMAT:
                set["status"] = "error";
                set["message"] = "Invalid 'set' command format. Use: set <parameter> <value>";
                break;
            case CommunicationManager::SetResult::UNKNOWN_PARAMETER:
                set["status"] = "error";
                set["message"] = "Unknown parameter";
                break;
            case CommunicationManager::SetResult::INVALID_VALUE:
                set["status"] = "error";
                set["message"] = "Invalid value. Expected: " + expected;
                break;
            case CommunicationManager::SetResult::OUT_OF_RANGE:
                set["status"] = "error";
                set["message"] = "Value out of range. Expected: " + expected;
                break;
        }
        serializeJson(doc, Serial);
        Serial.println();
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
void CommunicationManager::_printFlightStatus() {
    StaticJsonDocument<CommunicationManager::JSON_DOC_LARGE_SIZE> doc;

    if (isnan(_fc->state.attitude.roll) || isnan(_fc->state.attitude.pitch) || isnan(_fc->state.attitude.yaw)) {
        doc["error"] = "Attitude data is NaN. Check IMU connection.";
        serializeJson(doc, Serial);
        Serial.println();
        return;
    }

    JsonObject live_data = doc.createNestedObject("live_data");

    JsonObject attitude = live_data.createNestedObject("attitude");
    attitude["roll"] = _fc->state.attitude.roll;
    attitude["pitch"] = _fc->state.attitude.pitch;
    attitude["yaw"] = _fc->state.attitude.yaw;

    JsonObject status = live_data.createNestedObject("status");
    status["armed"] = _fc->state.isArmed;
    status["failsafe"] = _fc->state.isFailsafeActive;
    switch (_fc->state.currentFlightMode) {
        case ACRO_MODE: status["mode"] = "ACRO"; break;
        case ANGLE_MODE: status["mode"] = "ANGLE"; break;
        default: status["mode"] = "UNKNOWN"; break;
    }

    JsonArray motor_output = live_data.createNestedArray("motor_output");
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_output.add(_fc->state.motorOutputs[i]);
    }

    JsonArray receiver_channels = live_data.createNestedArray("receiver_channels");
    for (int i = 0; i < RECEIVER_CHANNEL_COUNT; i++) {
        receiver_channels.add(_fc->state.receiverChannels[i]);
    }

    serializeJson(doc, Serial);
    Serial.println();
}

// --- Public Methods ---

CommunicationManager::CommunicationManager(FlightController* fc) : _fc(fc) {}

void CommunicationManager::begin() {}

void CommunicationManager::update() {
    _handleSerialInput();
    if (_currentMode == OperatingMode::API && !_isSendingSettings && settings.enableLogging && millis() - _lastSerialLogTime >= settings.printIntervalMs) {
        _printFlightStatus();
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
        Serial.println(CommunicationManager::API_MODE_ACTIVATED_JSON);
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
        if (isApiMode) {
            StaticJsonDocument<CommunicationManager::JSON_DOC_SMALL_SIZE> doc;
            doc["error"] = "Unknown command";
            serializeJson(doc, Serial);
            Serial.println();
        } else {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        }
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

    Serial.println("  Complementary Filter:");
    Serial.println("    filter.comp_tau (Complementary Filter Time Constant)");
    Serial.println("    gyro.lpf_cutoff_freq (Gyroscope Low-Pass Filter Cutoff Frequency in Hz)");
    Serial.println("    accel.lpf_cutoff_freq (Accelerometer Low-Pass Filter Cutoff Frequency in Hz)");
    Serial.println("    gyro.lpf_stages (Number of Gyroscope LPF Stages, 1-5)");
    Serial.println("    accel.lpf_stages (Number of Accelerometer LPF Stages, 1-5)");
    Serial.println("    filter.sample_freq (Filter Sample Frequency in Hz)");

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
        case SettingType::INT:
            Serial.print(*static_cast<int*>(s.value));
            break;
        case SettingType::UINT8:
            Serial.print(*static_cast<uint8_t*>(s.value));
            break;
            case SettingType::ENUM_IBUS_PROTOCOL: Serial.println(getReceiverProtocolString(*(ReceiverProtocol*)s.value)); break;
            case SettingType::ENUM_IMU_PROTOCOL: Serial.println(getImuProtocolString(*(ImuProtocol*)s.value)); break;
            case SettingType::ENUM_LPF_BANDWIDTH: Serial.println(getLpfBandwidthString(*(LpfBandwidth*)s.value)); break;
            case SettingType::BOOL: Serial.println(getBoolString(*(bool*)s.value)); break;
            case SettingType::ENUM_DSHOT_MODE: Serial.println(getDShotModeString(*(dshot_mode_t*)s.value)); break;
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
    StaticJsonDocument<CommunicationManager::JSON_DOC_XLARGE_SIZE> jsonDoc; // Use a suitable size for your settings

    for (int i = 0; i < numSettings; ++i) {
        const Setting& s = settingsRegistry[i];
        switch (s.type) {
        case CommunicationManager::SettingType::FLOAT: jsonDoc[s.name] = *(float*)s.value / s.scaleFactor; break;
        case CommunicationManager::SettingType::INT: jsonDoc[s.name] = *(int*)s.value; break;
        case CommunicationManager::SettingType::UINT8: jsonDoc[s.name] = *(uint8_t*)s.value; break;
        case CommunicationManager::SettingType::UINT16: jsonDoc[s.name] = *(uint16_t*)s.value; break;
        case CommunicationManager::SettingType::BOOL: jsonDoc[s.name] = *(bool*)s.value; break;
        case CommunicationManager::SettingType::ENUM_IBUS_PROTOCOL: jsonDoc[s.name] = getReceiverProtocolString(*(ReceiverProtocol*)s.value); break;
        case CommunicationManager::SettingType::ENUM_IMU_PROTOCOL: jsonDoc[s.name] = getImuProtocolString(*(ImuProtocol*)s.value); break;
        case CommunicationManager::SettingType::ENUM_LPF_BANDWIDTH: jsonDoc[s.name] = getLpfBandwidthString(*(LpfBandwidth*)s.value); break;
        case CommunicationManager::SettingType::ENUM_DSHOT_MODE: jsonDoc[s.name] = getDShotModeString(*(dshot_mode_t*)s.value); break;
        }
    }
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
        String key = "rx.map.";
        String inputName = getFlightControlInputString((FlightControlInput)i);
        inputName.toLowerCase();
        key += inputName;
        jsonDoc[key] = settings.channelMapping.channel[i];
    }
    StaticJsonDocument<CommunicationManager::JSON_DOC_XLARGE_SIZE> outputDoc;
    outputDoc["settings"] = jsonDoc;
    serializeJson(outputDoc, Serial);
    Serial.println();
    _isSendingSettings = false;
}

void CommunicationManager::_handleGetCommand(String args, bool isApiMode) {
    for (int i = 0; i < numSettings; ++i) {
        const Setting& s = settingsRegistry[i];
        if (args.equalsIgnoreCase(s.name)) {
            String valueStr;
            bool isString = false;
            switch (s.type) {
                case CommunicationManager::SettingType::FLOAT: valueStr = String(*(float*)s.value / s.scaleFactor, 4); break;
                case CommunicationManager::SettingType::UINT16: valueStr = String(*(uint16_t*)s.value); break;
                case CommunicationManager::SettingType::UINT8: valueStr = getUint8String(*(uint8_t*)s.value); break;
                case SettingType::ENUM_IBUS_PROTOCOL: valueStr = getReceiverProtocolString(*(ReceiverProtocol*)s.value); isString = true; break;
                case SettingType::ENUM_IMU_PROTOCOL: valueStr = getImuProtocolString(*(ImuProtocol*)s.value); isString = true; break;
                case SettingType::ENUM_LPF_BANDWIDTH: valueStr = getLpfBandwidthString(*(LpfBandwidth*)s.value); isString = true; break;
                case SettingType::BOOL: valueStr = getBoolString(*(bool*)s.value); isString = true; break;
                case SettingType::ENUM_DSHOT_MODE: valueStr = getDShotModeString(*(dshot_mode_t*)s.value); isString = true; break;
            }
            _printGetResponse(args, valueStr, isApiMode, isString);
            return;
        }
    }
    if (isApiMode) { Serial.println(CommunicationManager::API_ERROR_UNKNOWN_PARAMETER_GET); } 
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
                case CommunicationManager::SettingType::FLOAT: {
                    float val;
                    result = _parseAndValidateFloat(valueStr, val, s.scaleFactor, expectedValue);
                    if (result == SetResult::SUCCESS) *(float*)s.value = val;
                    break;
                }
                case CommunicationManager::SettingType::UINT16: {
                    uint16_t val;
                    result = _parseAndValidateUint16(valueStr, val, expectedValue);
                    if (result == SetResult::SUCCESS) *(uint16_t*)s.value = val;
                    break;
                }
                case CommunicationManager::SettingType::ENUM_IBUS_PROTOCOL: {
                    ReceiverProtocol val;
                    result = _parseAndValidateReceiverProtocol(valueStr, val, expectedValue);
                    if (result == SetResult::SUCCESS) *(ReceiverProtocol*)s.value = val;
                    break;
                }
                case CommunicationManager::SettingType::ENUM_IMU_PROTOCOL: {
                    ImuProtocol val;
                    result = _parseAndValidateImuProtocol(valueStr, val, expectedValue);
                    if (result == SetResult::SUCCESS) *(ImuProtocol*)s.value = val;
                    break;
                }
                case CommunicationManager::SettingType::ENUM_LPF_BANDWIDTH: {
                    LpfBandwidth val;
                    result = _parseAndValidateLpfBandwidth(valueStr, val, expectedValue);
                    if (result == SetResult::SUCCESS) *(LpfBandwidth*)s.value = val;
                    break;
                }
                case CommunicationManager::SettingType::ENUM_DSHOT_MODE: {
                    dshot_mode_t val;
                    result = _parseAndValidateDShotMode(valueStr, val, expectedValue);
                    if (result == SetResult::SUCCESS) *(dshot_mode_t*)s.value = val;
                    break;
                }
            }
            _printSetResponse(param, valueStr, result, isApiMode, (s.type == CommunicationManager::SettingType::ENUM_DSHOT_MODE || s.type == CommunicationManager::SettingType::ENUM_IBUS_PROTOCOL || s.type == CommunicationManager::SettingType::ENUM_IMU_PROTOCOL || s.type == CommunicationManager::SettingType::ENUM_LPF_BANDWIDTH || s.type == CommunicationManager::SettingType::BOOL), expectedValue);
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
        StaticJsonDocument<CommunicationManager::JSON_DOC_SMALL_SIZE> doc;
        doc["version"] = FIRMWARE_VERSION;
        serializeJson(doc, Serial);
        Serial.println();
    } else {
        Serial.print("Firmware Version: ");
        Serial.println(FIRMWARE_VERSION);
    }
}