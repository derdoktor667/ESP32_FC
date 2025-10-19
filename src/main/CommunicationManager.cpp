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
#include <Arduino.h>
#include <ArduinoJson.h>

// --- Refactoring: Settings Registry ---

// --- Helper Functions for String Conversion ---
String CommunicationManager::_getReceiverProtocolString(ReceiverProtocol protocol) const
{
    if (protocol == PROTOCOL_IBUS)
        return "IBUS";
    if (protocol == PROTOCOL_PPM)
        return "PPM";
    return "UNKNOWN";
}

String CommunicationManager::_getImuProtocolString(ImuProtocol protocol) const
{
    if (protocol == IMU_MPU6050)
        return "MPU6050";
    return "UNKNOWN";
}

String CommunicationManager::_getLpfBandwidthString(LpfBandwidth bandwidth) const
{
    switch (bandwidth)
    {
    case LPF_256HZ_N_0MS:
        return "LPF_256HZ_N_0MS";
    case LPF_188HZ_N_2MS:
        return "LPF_188HZ_N_2MS";
    case LPF_98HZ_N_3MS:
        return "LPF_98HZ_N_3MS";
    case LPF_42HZ_N_5MS:
        return "LPF_42HZ_N_5MS";
    case LPF_20HZ_N_10MS:
        return "LPF_20HZ_N_10MS";
    case LPF_10HZ_N_13MS:
        return "LPF_10HZ_N_13MS";
    case LPF_5HZ_N_18MS:
        return "LPF_5HZ_N_18MS";
    default:
        return "UNKNOWN";
    }
}

String CommunicationManager::_getFlightControlInputString(FlightControlInput input) const
{
    switch (input)
    {
    case THROTTLE:
        return "THROTTLE";
    case ROLL:
        return "ROLL";
    case PITCH:
        return "PITCH";
    case YAW:
        return "YAW";
    case ARM_SWITCH:
        return "ARM_SWITCH";
    case FAILSAFE_SWITCH:
        return "FAILSAFE_SWITCH";
    case FLIGHT_MODE_SWITCH:
        return "FLIGHT_MODE_SWITCH";
    default:
        return "UNKNOWN";
    }
}

String CommunicationManager::_getDShotModeString(dshot_mode_t mode) const
{
    if (mode == DSHOT_OFF)
        return "DSHOT_OFF";
    if (mode == DSHOT150)
        return "DSHOT150";
    if (mode == DSHOT300)
        return "DSHOT300";
    if (mode == DSHOT600)
        return "DSHOT600";
    if (mode == DSHOT1200)
        return "DSHOT1200";
    return "UNKNOWN";
}

String CommunicationManager::_getImuRotationString(ImuRotation rotation) const
{
    switch (rotation)
    {
    case IMU_ROTATION_NONE:
        return "NONE";
    case IMU_ROTATION_90_DEG_CW:
        return "90_CW";
    case IMU_ROTATION_180_DEG_CW:
        return "180_CW";
    case IMU_ROTATION_270_DEG_CW:
        return "270_CW";
    case IMU_ROTATION_90_DEG_CCW:
        return "90_CCW";
    case IMU_ROTATION_180_DEG_CCW:
        return "180_CCW";
    case IMU_ROTATION_270_DEG_CCW:
        return "270_CCW";
    case IMU_ROTATION_FLIP:
        return "FLIP";
    default:
        return "UNKNOWN";
    }
}

String CommunicationManager::_getBoolString(bool value) const
{
    return value ? "true" : "false";
}

String CommunicationManager::_getUint8String(uint8_t value) const
{
    return String(value);
}

String CommunicationManager::_getULongString(unsigned long value) const
{
    return String(value);
}

// --- Refactoring: Settings Registry ---

const Setting CommunicationManager::settingsRegistry[] = {
    {"pid.roll.kp", SettingType::INT, &settings.pidRoll.kp, PID_SCALE_FACTOR},
    {"pid.roll.ki", SettingType::INT, &settings.pidRoll.ki, PID_SCALE_FACTOR},
    {"pid.roll.kd", SettingType::INT, &settings.pidRoll.kd, PID_SCALE_FACTOR},
    {"pid.pitch.kp", SettingType::INT, &settings.pidPitch.kp, PID_SCALE_FACTOR},
    {"pid.pitch.ki", SettingType::INT, &settings.pidPitch.ki, PID_SCALE_FACTOR},
    {"pid.pitch.kd", SettingType::INT, &settings.pidPitch.kd, PID_SCALE_FACTOR},
    {"pid.yaw.kp", SettingType::INT, &settings.pidYaw.kp, PID_SCALE_FACTOR},
    {"pid.yaw.ki", SettingType::INT, &settings.pidYaw.ki, PID_SCALE_FACTOR},
    {"pid.yaw.kd", SettingType::INT, &settings.pidYaw.kd, PID_SCALE_FACTOR},
    {"pid.integral_limit", SettingType::FLOAT, &settings.pidIntegralLimit, DEFAULT_SCALE_FACTOR},
    {"rates.angle", SettingType::FLOAT, &settings.rates.maxAngleRollPitch, DEFAULT_SCALE_FACTOR},
    {"rates.yaw", SettingType::FLOAT, &settings.rates.maxRateYaw, DEFAULT_SCALE_FACTOR},
    {"rates.acro", SettingType::FLOAT, &settings.rates.maxRateRollPitch, DEFAULT_SCALE_FACTOR},
    {"filter.comp_tau", SettingType::FLOAT, &settings.filter.complementaryFilterTau, DEFAULT_SCALE_FACTOR},
    {"gyro.lpf_cutoff_freq", SettingType::FLOAT, &settings.filter.gyroLpfCutoffFreq, DEFAULT_SCALE_FACTOR},
    {"accel.lpf_cutoff_freq", SettingType::FLOAT, &settings.filter.accelLpfCutoffFreq, DEFAULT_SCALE_FACTOR},
    {"gyro.lpf_stages", SettingType::UINT8, &settings.filter.gyroLpfStages, DEFAULT_SCALE_FACTOR},
    {"accel.lpf_stages", SettingType::UINT8, &settings.filter.accelLpfStages, DEFAULT_SCALE_FACTOR},
    {"filter.sample_freq", SettingType::FLOAT, &settings.filter.filterSampleFreq, DEFAULT_SCALE_FACTOR},
    {"gyro.notch.enable", SettingType::BOOL, &settings.filter.enableGyroNotchFilter, DEFAULT_SCALE_FACTOR},
    {"gyro.notch.freq", SettingType::FLOAT, &settings.filter.gyroNotchFreq, DEFAULT_SCALE_FACTOR},
    {"gyro.notch.q", SettingType::FLOAT, &settings.filter.gyroNotchQ, DEFAULT_SCALE_FACTOR},
    {"rx.min", SettingType::UINT16, &settings.receiver.ibusMinValue, DEFAULT_SCALE_FACTOR},
    {"rx.max", SettingType::UINT16, &settings.receiver.ibusMaxValue, DEFAULT_SCALE_FACTOR},
    {"rx.arming_threshold", SettingType::UINT16, &settings.receiver.armingThreshold, DEFAULT_SCALE_FACTOR},
    {"rx.failsafe_threshold", SettingType::UINT16, &settings.receiver.failsafeThreshold, DEFAULT_SCALE_FACTOR},
    {"rx.protocol", SettingType::ENUM_IBUS_PROTOCOL, &settings.receiverProtocol, DEFAULT_SCALE_FACTOR},
    {"imu.protocol", SettingType::ENUM_IMU_PROTOCOL, &settings.imuProtocol, DEFAULT_SCALE_FACTOR},
    {"imu.lpf", SettingType::ENUM_LPF_BANDWIDTH, &settings.imuLpfBandwidth, DEFAULT_SCALE_FACTOR},
    {"imu.rotation", SettingType::ENUM_IMU_ROTATION, &settings.imuRotation, DEFAULT_SCALE_FACTOR},
    {"motor.idle_speed", SettingType::FLOAT, &settings.motorIdleSpeedPercent, DEFAULT_SCALE_FACTOR},
    {"motor.dshot_mode", SettingType::ENUM_DSHOT_MODE, &settings.dshotMode, DEFAULT_SCALE_FACTOR},
    {"enforce_loop_time", SettingType::BOOL, &settings.enforceLoopTime, DEFAULT_SCALE_FACTOR},
    {"cal.mpu_readings", SettingType::INT, &settings.calibration.mpuCalibrationReadings, DEFAULT_SCALE_FACTOR},
    {"cal.accel_z_g", SettingType::FLOAT, &settings.calibration.accelZGravity, DEFAULT_SCALE_FACTOR},
    {"log.print_interval", SettingType::ULONG, &settings.printIntervalMs, DEFAULT_SCALE_FACTOR},
    {"log.enable", SettingType::BOOL, &settings.enableLogging, DEFAULT_SCALE_FACTOR},
};
const int CommunicationManager::numSettings = sizeof(CommunicationManager::settingsRegistry) / sizeof(Setting);

// --- Helper Functions for Parsing and Validation ---

SetResult CommunicationManager::_parseAndValidateFloat(const String &valueStr, float &outValue, float scaleFactor, String &expectedValue) const
{
    float val = valueStr.toFloat();
    if (valueStr.length() > 0 && val == 0.0f && valueStr != "0" && valueStr != "0.0")
    {
        expectedValue = "float";
        return SetResult::INVALID_VALUE;
    }
    outValue = val * scaleFactor;
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateInt(const String &valueStr, int &outValue, String &expectedValue) const
{
    long val = valueStr.toInt();
    if (valueStr.length() > 0 && val == 0 && valueStr != "0")
    {
        expectedValue = "integer";
        return SetResult::INVALID_VALUE;
    }
    // No explicit range check for int, as it can be negative.
    // If specific int ranges are needed, they should be added here.
    outValue = (int)val;
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateUint16(const String &valueStr, uint16_t &outValue, String &expectedValue) const
{
    long val = valueStr.toInt();
    if (valueStr.length() > 0 && val == 0 && valueStr != "0")
    {
        expectedValue = "integer";
        return SetResult::INVALID_VALUE;
    }
    else if (val < 0 || val > MAX_UINT16_VALUE)
    { // uint16_t range
        expectedValue = "0-" + String(MAX_UINT16_VALUE);
        return SetResult::OUT_OF_RANGE;
    }
    outValue = (uint16_t)val;
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateULong(const String &valueStr, unsigned long &outValue, String &expectedValue) const
{
    // toULong() returns 0 if no valid conversion could be performed.
    // We need to check if the string was actually "0" or if it was invalid.
    unsigned long val = strtoul(valueStr.c_str(), NULL, 10);
    if (valueStr.length() > 0 && val == 0 && valueStr != "0")
    {
        expectedValue = "unsigned long integer";
        return SetResult::INVALID_VALUE;
    }
    outValue = val;
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateReceiverProtocol(const String &valueStr, ReceiverProtocol &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("IBUS"))
        outValue = PROTOCOL_IBUS;
    else if (valueStr.equalsIgnoreCase("PPM"))
        outValue = PROTOCOL_PPM;
    else
    {
        expectedValue = "IBUS, PPM";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateImuProtocol(const String &valueStr, ImuProtocol &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("MPU6050"))
        outValue = IMU_MPU6050;
    else
    {
        expectedValue = "MPU6050";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateLpfBandwidth(const String &valueStr, LpfBandwidth &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("LPF_256HZ_N_0MS"))
        outValue = LPF_256HZ_N_0MS;
    else if (valueStr.equalsIgnoreCase("LPF_188HZ_N_2MS"))
        outValue = LPF_188HZ_N_2MS;
    else if (valueStr.equalsIgnoreCase("LPF_98HZ_N_3MS"))
        outValue = LPF_98HZ_N_3MS;
    else if (valueStr.equalsIgnoreCase("LPF_42HZ_N_5MS"))
        outValue = LPF_42HZ_N_5MS;
    else if (valueStr.equalsIgnoreCase("LPF_20HZ_N_10MS"))
        outValue = LPF_20HZ_N_10MS;
    else if (valueStr.equalsIgnoreCase("LPF_10HZ_N_13MS"))
        outValue = LPF_10HZ_N_13MS;
    else if (valueStr.equalsIgnoreCase("LPF_5HZ_N_18MS"))
        outValue = LPF_5HZ_N_18MS;
    else
    {
        expectedValue = "LPF_256HZ_N_0MS, LPF_188HZ_N_2MS, LPF_98HZ_N_3MS, LPF_42HZ_N_5MS, LPF_20HZ_N_10MS, LPF_10HZ_N_13MS, LPF_5HZ_N_18MS";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateImuRotation(const String &valueStr, ImuRotation &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("NONE"))
        outValue = IMU_ROTATION_NONE;
    else if (valueStr.equalsIgnoreCase("90_CW"))
        outValue = IMU_ROTATION_90_DEG_CW;
    else if (valueStr.equalsIgnoreCase("180_CW"))
        outValue = IMU_ROTATION_180_DEG_CW;
    else if (valueStr.equalsIgnoreCase("270_CW"))
        outValue = IMU_ROTATION_270_DEG_CW;
    else if (valueStr.equalsIgnoreCase("90_CCW"))
        outValue = IMU_ROTATION_90_DEG_CCW;
    else if (valueStr.equalsIgnoreCase("180_CCW"))
        outValue = IMU_ROTATION_180_DEG_CCW;
    else if (valueStr.equalsIgnoreCase("270_CCW"))
        outValue = IMU_ROTATION_270_DEG_CCW;
    else if (valueStr.equalsIgnoreCase("FLIP"))
        outValue = IMU_ROTATION_FLIP;
    else
    {
        expectedValue = "NONE, 90_CW, 180_CW, 270_CW, 90_CCW, 180_CCW, 270_CCW, FLIP";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateDShotMode(const String &valueStr, dshot_mode_t &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("DSHOT_OFF"))
        outValue = DSHOT_OFF;
    else if (valueStr.equalsIgnoreCase("DSHOT150"))
        outValue = DSHOT150;
    else if (valueStr.equalsIgnoreCase("DSHOT300"))
        outValue = DSHOT300;
    else if (valueStr.equalsIgnoreCase("DSHOT600"))
        outValue = DSHOT600;
    else if (valueStr.equalsIgnoreCase("DSHOT1200"))
        outValue = DSHOT1200;
    else
    {
        expectedValue = "DSHOT_OFF, DSHOT150, DSHOT300, DSHOT600, DSHOT1200";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateBool(const String &valueStr, bool &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("true"))
        outValue = true;
    else if (valueStr.equalsIgnoreCase("false"))
        outValue = false;
    else
    {
        expectedValue = "true, false";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateRxChannelMap(const String &param, const String &valueStr, int &outValue, String &expectedValue) const
{
    String inputName = param.substring(RX_MAP_PREFIX_LENGTH);
    int channelValue = valueStr.toInt();

    if (valueStr.length() > 0 && channelValue == 0 && valueStr != "0")
    {
        expectedValue = "integer";
        return SetResult::INVALID_VALUE;
    }
    else if (channelValue < 0 || channelValue >= RECEIVER_CHANNEL_COUNT)
    {
        expectedValue = "0-" + String(RECEIVER_CHANNEL_COUNT - 1);
        return SetResult::OUT_OF_RANGE;
    }
    else
    {
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
        {
            if (inputName.equalsIgnoreCase(_getFlightControlInputString((FlightControlInput)i)))
            {
                outValue = channelValue;
                return SetResult::SUCCESS;
            }
        }
    }
    return SetResult::UNKNOWN_PARAMETER;
}

// --- Command Helpers for Get/Set ---

void CommunicationManager::_printGetResponse(const String &param, const String &value, bool isApiMode, bool isString) const
{
    if (isApiMode)
    {
        StaticJsonDocument<JSON_DOC_MEDIUM_SIZE> doc;
        JsonObject get = doc.createNestedObject("get");
        if (isString)
        {
            get[param] = value;
        }
        else
        {
            get[param] = serialized(value);
        }
        serializeJson(doc, Serial);
        Serial.println();
    }
    else
    {
        Serial.println(value);
    }
}

void CommunicationManager::_printSetResponse(const String &param, const String &value, SetResult result, bool isApiMode, bool isString, const String &expected) const
{
    if (isApiMode)
    {
        StaticJsonDocument<JSON_DOC_LARGE_SIZE> doc;
        JsonObject set = doc.createNestedObject("set");
        if (isString)
        {
            set[param] = value;
        }
        else
        {
            set[param] = serialized(value);
        }

        switch (result)
        {
        case SetResult::SUCCESS:
            set["status"] = "success";
            break;
        case SetResult::INVALID_FORMAT:
            set["status"] = "error";
            set["message"] = "Invalid 'set' command format. Use: set <parameter> <value>";
            break;
        case SetResult::UNKNOWN_PARAMETER:
            set["status"] = "error";
            set["message"] = "Unknown parameter";
            break;
        case SetResult::INVALID_VALUE:
            set["status"] = "error";
            set["message"] = "Invalid value. Expected: " + expected;
            break;
        case SetResult::OUT_OF_RANGE:
            set["status"] = "error";
            set["message"] = "Value out of range. Expected: " + expected;
            break;
        }
        serializeJson(doc, Serial);
        Serial.println();
    }
    else
    {
        switch (result)
        {
        case SetResult::SUCCESS:
            Serial.print("Set ");
            Serial.print(param);
            Serial.print(" to ");
            Serial.println(value);
            break;
        case SetResult::INVALID_FORMAT:
            Serial.println("Invalid 'set' format. Use: set <parameter> <value>");
            break;
        case SetResult::UNKNOWN_PARAMETER:
            Serial.print("Unknown parameter: ");
            Serial.println(param);
            break;
        case SetResult::INVALID_VALUE:
            Serial.print("Invalid value '");
            Serial.print(value);
            Serial.print("' for parameter '");
            Serial.print(param);
            Serial.print("'. Expected: ");
            Serial.println(expected);
            break;
        case SetResult::OUT_OF_RANGE:
            Serial.print("Value '");
            Serial.print(value);
            Serial.print("' for parameter '");
            Serial.print(param);
            Serial.print("' is out of range. Expected: ");
            Serial.println(expected);
            break;
        }
    }
}

// Helper function to print live flight status data to serial, formatted as JSON for API mode.
void CommunicationManager::_printFlightStatus() const
{
    StaticJsonDocument<JSON_DOC_LARGE_SIZE> doc;

    if (isnan(_fc->state.attitude.roll) || isnan(_fc->state.attitude.pitch) || isnan(_fc->state.attitude.yaw))
    {
        doc["error"] = "Attitude data is NaN. Check IMU connection.";
        serializeJson(doc, Serial);
        Serial.println();
        return;
    }

    JsonObject live_data = doc.createNestedObject("live_data");

    JsonObject attitude = live_data.createNestedObject("attitude");
    char buffer[ATTITUDE_BUFFER_SIZE]; // Buffer to hold the formatted float string
    snprintf(buffer, sizeof(buffer), "%.2f", _fc->state.attitude.roll);
    attitude["roll"] = buffer;
    snprintf(buffer, sizeof(buffer), "%.2f", _fc->state.attitude.pitch);
    attitude["pitch"] = buffer;
    snprintf(buffer, sizeof(buffer), "%.2f", _fc->state.attitude.yaw);
    attitude["yaw"] = buffer;

    JsonObject status = live_data.createNestedObject("status");
    status["armed"] = _fc->state.isArmed;
    status["failsafe"] = _fc->state.isFailsafeActive;
    switch (_fc->state.currentFlightMode)
    {
    case ACRO_MODE:
        status["mode"] = "ACRO";
        break;
    case ANGLE_MODE:
        status["mode"] = "ANGLE";
        break;
    default:
        status["mode"] = "UNKNOWN";
        break;
    }

    JsonArray motor_output = live_data.createNestedArray("motor_output");
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        motor_output.add(_fc->state.motorOutputs[i]);
    }

    JsonArray receiver_channels = live_data.createNestedArray("receiver_channels");
    for (int i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
    {
        receiver_channels.add((int)_fc->state.receiverChannels[i]);
    }

    serializeJson(doc, Serial);
    Serial.println();
}

// --- Public Methods ---

CommunicationManager::CommunicationManager(FlightController *fc) : _fc(fc) {}

void CommunicationManager::initializeCommunication() {}

void CommunicationManager::processCommunication()
{
    _handleSerialInput();
    if (_currentMode == OperatingMode::API && !_isSendingSettings && settings.enableLogging && millis() - _lastSerialLogTime >= settings.printIntervalMs)
    {
        _printFlightStatus();
        _lastSerialLogTime = millis();
    }
}

// --- Private Methods: Main Logic ---

void CommunicationManager::_handleSerialInput()
{
    if (Serial.available() == 0)
        return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0)
        return;

    switch (_currentMode)
    {
    case OperatingMode::FLIGHT:
        _handleFlightModeInput(input);
        break;
    case OperatingMode::CLI:
    {
        String commandName = (input.indexOf(' ') != -1) ? input.substring(0, input.indexOf(' ')) : input;
        if (commandName.equalsIgnoreCase("exit"))
        {
            _currentMode = OperatingMode::FLIGHT;
            Serial.println("--- CLI  Deactivated ---");
            return;
        }
        _executeCommand(input, false);
        if (!commandName.equalsIgnoreCase("save") && !commandName.equalsIgnoreCase("reboot") && !commandName.equalsIgnoreCase("reset"))
        {
            Serial.print("ESP32_FC > ");
        }
        break;
    }
    case OperatingMode::API:
        _executeCommand(input, true);
        break;
    }
}

void CommunicationManager::_handleFlightModeInput(const String &input)
{
    if (input.equalsIgnoreCase("cli"))
    {
        _currentMode = OperatingMode::CLI;
        settings.enableLogging = false;
        Serial.println("--- CLI  Activated ---");
        Serial.print("ESP32_FC > ");
    }
    else if (input.equalsIgnoreCase("api"))
    {
        _currentMode = OperatingMode::API;
        settings.enableLogging = true;
        Serial.println(API_MODE_ACTIVATED_JSON);
    }
}

void CommunicationManager::_executeCommand(String command, bool isApiMode)
{
    String commandName = "";
    String commandArgs = "";
    int firstSpace = command.indexOf(' ');
    if (firstSpace != -1)
    {
        commandName = command.substring(0, firstSpace);
        commandArgs = command.substring(firstSpace + 1);
    }
    else
    {
        commandName = command;
    }
    commandName.toLowerCase(); // Only lowercase the command name itself

    if (commandName.equals("get") || commandName.equals("set") || commandName.equals("dump") || commandName.equals("get_settings"))
    {
        _handleSettingsCommand(commandName, commandArgs, isApiMode);
    }
    else if (commandName.equals("save") || commandName.equals("reset") || commandName.equals("reboot") || commandName.equals("status") || commandName.equals("version"))
    {
        _handleSystemCommand(commandName, isApiMode);
    }
    else if (commandName.equals("calibrate_imu") || commandName.equals("help"))
    {
        _handleUtilityCommand(commandName, isApiMode);
    }
    else
    {
        if (isApiMode)
        {
            StaticJsonDocument<JSON_DOC_SMALL_SIZE> doc;
            doc["error"] = "Unknown command";
            serializeJson(doc, Serial);
            Serial.println();
        }
        else
        {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        }
    }
}

// --- New Helper Methods for Command Handling ---

void CommunicationManager::_handleSettingsCommand(String commandName, String commandArgs, bool isApiMode)
{
    if (commandName.equals("get"))
        _handleGetCommand(commandArgs, isApiMode);
    else if (commandName.equals("set"))
        _handleSetCommand(commandArgs, isApiMode);
    else if (commandName.equals("dump") && !isApiMode)
        _handleDumpCommand();
    else if (commandName.equals("get_settings") && isApiMode)
        _handleDumpJsonCommand();
    else
    {
        if (isApiMode)
        {
            StaticJsonDocument<JSON_DOC_SMALL_SIZE> doc;
            doc["error"] = "Unknown command";
            serializeJson(doc, Serial);
            Serial.println();
        }
        else
        {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        }
    }
}

void CommunicationManager::_handleSystemCommand(String commandName, bool isApiMode)
{
    if (commandName.equals("save"))
    {
        if (!isApiMode)
            Serial.println("INFO: Settings saved. Rebooting...");
        saveSettings();
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    }
    else if (commandName.equals("reset"))
    {
        if (!isApiMode)
            Serial.println("INFO: All settings have been reset to their default values and saved.");
        settings = FlightControllerSettings();
        saveSettings();
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    }
    else if (commandName.equals("reboot"))
    {
        if (!isApiMode)
            Serial.println("Rebooting...");
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    }
    else if (commandName.equals("status") && !isApiMode)
    {
        _handleStatusCommand();
    }
    else if (commandName.equals("version"))
    {
        _handleVersionCommand();
    }
    else
    {
        if (isApiMode)
        {
            StaticJsonDocument<JSON_DOC_SMALL_SIZE> doc;
            doc["error"] = "Unknown command";
            serializeJson(doc, Serial);
            Serial.println();
        }
        else
        {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        }
    }
}

void CommunicationManager::_handleUtilityCommand(String commandName, bool isApiMode)
{
    if (commandName.equals("calibrate_imu"))
    {
        if (!isApiMode)
            Serial.println("INFO: IMU calibration requested.");
        _fc->requestImuCalibration();
    }
    else if (commandName.equals("help") && !isApiMode)
    {
        _printCliHelp();
    }
    else
    {
        if (isApiMode)
        {
            StaticJsonDocument<JSON_DOC_SMALL_SIZE> doc;
            doc["error"] = "Unknown command";
            serializeJson(doc, Serial);
            Serial.println();
        }
        else
        {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        }
    }
}

// --- Refactored Command Implementations ---

void CommunicationManager::_printCliHelp()
{
    // --- Helper function to print a formatted line ---
    auto printFormattedLine = [](const String &item, const String &description)
    {
        int itemPadding = 25 - item.length();
        if (itemPadding < 1)
            itemPadding = 1;
        Serial.print("  ");
        Serial.print(item);
        for (int i = 0; i < itemPadding; ++i)
            Serial.print(" ");
        Serial.print("- ");
        Serial.println(description);
    };

    // --- Header ---
    Serial.println("\n--- ESP32 Flight Controller CLI Help ---");
    Serial.println("Provides commands to get, set, and manage flight controller settings.");

    // --- Commands ---
    Serial.println("\n--- Commands ---");
    printFormattedLine("get <param>", "Get the current value of a setting.");
    printFormattedLine("set <param> <value>", "Set a new value for a setting.");
    printFormattedLine("dump", "Display all current settings.");
    printFormattedLine("save", "Save current settings to flash and reboot.");
    printFormattedLine("reset", "Reset all settings to default and reboot.");
    printFormattedLine("reboot", "Reboot the ESP32.");
    printFormattedLine("calibrate_imu", "Start IMU calibration sequence.");
    printFormattedLine("status", "Display system status and metrics.");
    printFormattedLine("version", "Display firmware version.");
    printFormattedLine("help", "Display this help message.");
    printFormattedLine("exit", "Exit CLI mode and return to flight mode.");

    // --- Settings ---
    Serial.println("\n--- Available Settings ---");
    String currentGroup = "";

    for (int i = 0; i < numSettings; ++i)
    {
        const Setting &s = settingsRegistry[i];
        String settingName(s.name);

        // Print group header if it changes
        int dotIndex = settingName.indexOf('.');
        String group = (dotIndex != -1) ? settingName.substring(0, dotIndex) : settingName;
        if (group != currentGroup)
        {
            currentGroup = group;
            String groupHeader = "\n  " + group + " Settings:";
            groupHeader.toUpperCase();
            Serial.println(groupHeader);
        }

        // Prepare description string with type and valid values
        String description = "";
        String expectedValue = "";
        switch (s.type)
        {
        case SettingType::FLOAT:
            description += "[float] ";
            break;
        case SettingType::INT:
            description += "[int]   ";
            break;
        case SettingType::UINT8:
            description += "[uint8] ";
            break;
        case SettingType::UINT16:
            description += "[uint16]";
            break;
        case SettingType::BOOL:
            description += "[bool]  ";
            description += "Values: true, false";
            break;
        case SettingType::ENUM_IBUS_PROTOCOL:
        {
            description += "[enum]  ";
            ReceiverProtocol dummy;
            _parseAndValidateReceiverProtocol("invalid", dummy, expectedValue);
            description += "Values: " + expectedValue;
            break;
        }
        case SettingType::ENUM_IMU_PROTOCOL:
        {
            description += "[enum]  ";
            ImuProtocol dummy;
            _parseAndValidateImuProtocol("invalid", dummy, expectedValue);
            description += "Values: " + expectedValue;
            break;
        }
        case SettingType::ENUM_LPF_BANDWIDTH:
        {
            description += "[enum]  ";
            LpfBandwidth dummy;
            _parseAndValidateLpfBandwidth("invalid", dummy, expectedValue);
            description += "Values: " + expectedValue;
            break;
        }
        case SettingType::ENUM_IMU_ROTATION:
        {
            description += "[enum]  ";
            ImuRotation dummy;
            _parseAndValidateImuRotation("invalid", dummy, expectedValue);
            description += "Values: " + expectedValue;
            break;
        }
        case SettingType::ENUM_DSHOT_MODE:
        {
            description += "[enum]  ";
            dshot_mode_t dummy;
            _parseAndValidateDShotMode("invalid", dummy, expectedValue);
            description += "Values: " + expectedValue;
            break;
        }
        default:
            break;
        }

        printFormattedLine("  " + settingName, description);
    }

    // Manual entry for channel mapping
    Serial.println("\n  RX Settings (Channel Mapping):");
    printFormattedLine("  rx.map.<input> <ch>", "[int]   Map a flight control input to a receiver channel (0-15).");
    Serial.println("\n----------------------------------------\n");
}

void CommunicationManager::_handleDumpCommand()
{
    Serial.println("--- Current Flight Controller Settings ---");
    for (int i = 0; i < numSettings; ++i)
    {
        const Setting &s = settingsRegistry[i];
        Serial.print(s.name);
        Serial.print(": ");
        switch (s.type)
        {
        case SettingType::FLOAT:
            Serial.println(*(float *)s.value / s.scaleFactor, 4);
            break;
        case SettingType::INT:
            if (s.scaleFactor == PID_SCALE_FACTOR)
            {
                Serial.println(*(int *)s.value / 10);
            }
            else
            {
                Serial.println(*(int *)s.value);
            }
            break;
        case SettingType::UINT8:
            Serial.println(*(uint8_t *)s.value);
            break;
        case SettingType::UINT16:
            Serial.println(*(uint16_t *)s.value);
            break;
        case SettingType::ULONG:
            Serial.println(*(unsigned long *)s.value);
            break;
        case SettingType::ENUM_IBUS_PROTOCOL:
            Serial.println(_getReceiverProtocolString(*(ReceiverProtocol *)s.value));
            break;
        case SettingType::ENUM_IMU_PROTOCOL:
            Serial.println(_getImuProtocolString(*(ImuProtocol *)s.value));
            break;
        case SettingType::ENUM_LPF_BANDWIDTH:
            Serial.println(_getLpfBandwidthString(*(LpfBandwidth *)s.value));
            break;
        case SettingType::ENUM_IMU_ROTATION:
            Serial.println(_getImuRotationString(*(ImuRotation *)s.value));
            break;
        case SettingType::BOOL:
            Serial.println(_getBoolString(*(bool *)s.value));
            break;
        case SettingType::ENUM_DSHOT_MODE:
            Serial.println(_getDShotModeString(*(dshot_mode_t *)s.value));
            break;
        }
    }
    Serial.println("\n--- Receiver Channel Mapping ---");
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
    {
        Serial.print("  ");
        Serial.print(_getFlightControlInputString((FlightControlInput)i));
        Serial.print(": ");
        Serial.println(settings.channelMapping.channel[i]);
    }
    Serial.println("----------------------------------------");
}

void CommunicationManager::_handleDumpJsonCommand()
{
    _isSendingSettings = true;
    StaticJsonDocument<JSON_DOC_XLARGE_SIZE> jsonDoc; // Use a suitable size for your settings

    for (int i = 0; i < numSettings; ++i)
    {
        const Setting &s = settingsRegistry[i];
        switch (s.type)
        {
        case SettingType::FLOAT:
            jsonDoc[s.name] = *(float *)s.value / s.scaleFactor;
            break;
        case SettingType::INT:
            if (s.scaleFactor == PID_SCALE_FACTOR)
            {
                jsonDoc[s.name] = *(int *)s.value / 10;
            }
            else
            {
                jsonDoc[s.name] = *(int *)s.value;
            }
            break;
        case SettingType::UINT8:
            jsonDoc[s.name] = *(uint8_t *)s.value;
            break;
        case SettingType::UINT16:
            jsonDoc[s.name] = *(uint16_t *)s.value;
            break;
        case SettingType::ULONG:
            jsonDoc[s.name] = *(unsigned long *)s.value;
            break;
        case SettingType::BOOL:
            jsonDoc[s.name] = *(bool *)s.value;
            break;
        case SettingType::ENUM_IBUS_PROTOCOL:
            jsonDoc[s.name] = _getReceiverProtocolString(*(ReceiverProtocol *)s.value);
            break;
        case SettingType::ENUM_IMU_PROTOCOL:
            jsonDoc[s.name] = _getImuProtocolString(*(ImuProtocol *)s.value);
            break;
        case SettingType::ENUM_LPF_BANDWIDTH:
            jsonDoc[s.name] = _getLpfBandwidthString(*(LpfBandwidth *)s.value);
            break;
        case SettingType::ENUM_IMU_ROTATION:
            jsonDoc[s.name] = _getImuRotationString(*(ImuRotation *)s.value);
            break;
        case SettingType::ENUM_DSHOT_MODE:
            jsonDoc[s.name] = _getDShotModeString(*(dshot_mode_t *)s.value);
            break;
        }
    }
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
    {
        String key = "rx.map.";
        String inputName = _getFlightControlInputString((FlightControlInput)i);
        inputName.toLowerCase();
        key += inputName;
        jsonDoc[key] = settings.channelMapping.channel[i];
    }
    StaticJsonDocument<JSON_DOC_XLARGE_SIZE> outputDoc;
    outputDoc["settings"] = jsonDoc;
    serializeJson(outputDoc, Serial);
    Serial.println();
    _isSendingSettings = false;
}

void CommunicationManager::_handleGetCommand(String args, bool isApiMode)
{
    for (int i = 0; i < numSettings; ++i)
    {
        const Setting &s = settingsRegistry[i];
        if (args.equalsIgnoreCase(s.name))
        {
            String valueStr;
            bool isString = false;
            switch (s.type)
            {
            case SettingType::FLOAT:
                valueStr = String(*(float *)s.value / s.scaleFactor, 4);
                break;
            case SettingType::INT:
                if (s.scaleFactor == PID_SCALE_FACTOR)
                {
                    valueStr = String(*(int *)s.value / 10);
                }
                else
                {
                    valueStr = String(*(int *)s.value);
                }
                break;
            case SettingType::UINT8:
                valueStr = _getUint8String(*(uint8_t *)s.value);
                break;
            case SettingType::UINT16:
                valueStr = String(*(uint16_t *)s.value);
                break;
            case SettingType::ULONG:
                valueStr = _getULongString(*(unsigned long *)s.value);
                break;
            case SettingType::ENUM_IBUS_PROTOCOL:
                valueStr = _getReceiverProtocolString(*(ReceiverProtocol *)s.value);
                isString = true;
                break;
            case SettingType::ENUM_IMU_PROTOCOL:
                valueStr = _getImuProtocolString(*(ImuProtocol *)s.value);
                isString = true;
                break;
            case SettingType::ENUM_LPF_BANDWIDTH:
                valueStr = _getLpfBandwidthString(*(LpfBandwidth *)s.value);
                isString = true;
                break;
            case SettingType::ENUM_IMU_ROTATION:
                valueStr = _getImuRotationString(*(ImuRotation *)s.value);
                isString = true;
                break;
            case SettingType::BOOL:
                valueStr = _getBoolString(*(bool *)s.value);
                isString = true;
                break;
            case SettingType::ENUM_DSHOT_MODE:
                valueStr = _getDShotModeString(*(dshot_mode_t *)s.value);
                isString = true;
                break;
            }
            _printGetResponse(args, valueStr, isApiMode, isString);
            return;
        }
    }
    if (isApiMode)
    {
        Serial.println(API_ERROR_UNKNOWN_PARAMETER_GET);
    }
    else
    {
        Serial.println("Unknown parameter for 'get'.");
    }
}

void CommunicationManager::_handleSetCommand(String args, bool isApiMode)
{
    int lastSpace = args.lastIndexOf(' ');
    if (lastSpace == -1)
    {
        _printSetResponse("", "", SetResult::INVALID_FORMAT, isApiMode, false, "");
        return;
    }
    String param = args.substring(0, lastSpace);
    String valueStr = args.substring(lastSpace + 1);

    for (int i = 0; i < numSettings; ++i)
    {
        const Setting &s = settingsRegistry[i];
        if (param.equalsIgnoreCase(s.name))
        {
            SetResult result = SetResult::SUCCESS;
            String expectedValue = "";

            switch (s.type)
            {
            case SettingType::FLOAT:
            {
                float val;
                result = _parseAndValidateFloat(valueStr, val, s.scaleFactor, expectedValue);
                if (result == SetResult::SUCCESS)
                    *(float *)s.value = val;
                break;
            }
            case SettingType::INT:
            {
                int val;
                result = _parseAndValidateInt(valueStr, val, expectedValue);
                if (result == SetResult::SUCCESS)
                {
                    if (s.scaleFactor == PID_SCALE_FACTOR)
                    {
                        *(int *)s.value = val * 10;
                    }
                    else
                    {
                        *(int *)s.value = val;
                    }
                }
                break;
            }
            case SettingType::UINT8:
            {
                uint8_t val;
                long tempVal = valueStr.toInt();
                if (valueStr.length() > 0 && tempVal == 0 && valueStr != "0")
                {
                    expectedValue = "integer";
                    result = SetResult::INVALID_VALUE;
                }
                else if (tempVal < 0 || tempVal > MAX_UINT8_VALUE)
                { // UINT8 range
                    expectedValue = "0-" + String(MAX_UINT8_VALUE);
                    result = SetResult::OUT_OF_RANGE;
                }
                else
                {
                    val = (uint8_t)tempVal;
                    result = SetResult::SUCCESS;
                }

                if (result == SetResult::SUCCESS)
                    *(uint8_t *)s.value = val;
                break;
            }
            case SettingType::UINT16:
            {
                uint16_t val;
                result = _parseAndValidateUint16(valueStr, val, expectedValue);
                if (result == SetResult::SUCCESS)
                    *(uint16_t *)s.value = val;
                break;
            }
            case SettingType::ULONG:
            {
                unsigned long val;
                result = _parseAndValidateULong(valueStr, val, expectedValue);
                if (result == SetResult::SUCCESS)
                    *(unsigned long *)s.value = val;
                break;
            }
            case SettingType::BOOL:
            {
                bool val;
                result = _parseAndValidateBool(valueStr, val, expectedValue);
                if (result == SetResult::SUCCESS)
                {
                    *(bool *)s.value = val;
                    // If gyro.notch.enable is changed, update the notch filter state
                    if (param.equalsIgnoreCase("gyro.notch.enable"))
                    {
                        _fc->getAttitudeEstimator().updateNotchFilterState();
                    }
                }
                break;
            }
            case SettingType::ENUM_IBUS_PROTOCOL:
            {
                ReceiverProtocol val;
                result = _parseAndValidateReceiverProtocol(valueStr, val, expectedValue);
                if (result == SetResult::SUCCESS)
                    *(ReceiverProtocol *)s.value = val;
                break;
            }
            case SettingType::ENUM_IMU_PROTOCOL:
            {
                ImuProtocol val;
                result = _parseAndValidateImuProtocol(valueStr, val, expectedValue);
                if (result == SetResult::SUCCESS)
                    *(ImuProtocol *)s.value = val;
                break;
            }
            case SettingType::ENUM_LPF_BANDWIDTH:
            {
                LpfBandwidth val;
                result = _parseAndValidateLpfBandwidth(valueStr, val, expectedValue);
                if (result == SetResult::SUCCESS)
                    *(LpfBandwidth *)s.value = val;
                break;
            }
            case SettingType::ENUM_IMU_ROTATION:
            {
                ImuRotation val;
                result = _parseAndValidateImuRotation(valueStr, val, expectedValue);
                if (result == SetResult::SUCCESS)
                    *(ImuRotation *)s.value = val;
                break;
            }
            case SettingType::ENUM_DSHOT_MODE:
            {
                dshot_mode_t val;
                result = _parseAndValidateDShotMode(valueStr, val, expectedValue);
                if (result == SetResult::SUCCESS)
                    *(dshot_mode_t *)s.value = val;
                break;
            }
            case SettingType::STRING:
            case SettingType::ENUM_RX_CHANNEL_MAP:
                result = SetResult::UNKNOWN_PARAMETER; // Or a more specific error
                break;
            }
            _printSetResponse(param, valueStr, result, isApiMode, (s.type == SettingType::ENUM_DSHOT_MODE || s.type == SettingType::ENUM_IBUS_PROTOCOL || s.type == SettingType::ENUM_IMU_PROTOCOL || s.type == SettingType::ENUM_LPF_BANDWIDTH || s.type == SettingType::ENUM_IMU_ROTATION || s.type == SettingType::BOOL), expectedValue);
            return;
        }
    }

    // Handle rx.map settings
    if (param.startsWith("rx.map."))
    {
        int channelValue;
        String expectedValue = "";
        SetResult result = _parseAndValidateRxChannelMap(param, valueStr, channelValue, expectedValue);

        if (result == SetResult::SUCCESS)
        {
            String inputName = param.substring(RX_MAP_PREFIX_LENGTH);
            for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
            {
                if (inputName.equalsIgnoreCase(_getFlightControlInputString((FlightControlInput)i)))
                {
                    settings.channelMapping.channel[i] = channelValue;
                    break;
                }
            }
        }
        _printSetResponse(param, valueStr, result, isApiMode, false, expectedValue);
        return;
    }

    _printSetResponse(param, valueStr, SetResult::UNKNOWN_PARAMETER, isApiMode, false, "");
}

void CommunicationManager::_handleStatusCommand() const
{
    Serial.println("--- System Status ---");
    Serial.print("Target Loop Time (us): ");
    Serial.println(TARGET_LOOP_TIME_US);
    Serial.print("Actual Loop Time (us): ");
    Serial.println(_fc->state.loopTimeUs);
    Serial.print("CPU Load (%): ");
    Serial.println(_fc->state.cpuLoad, 2);
    Serial.print("Battery Voltage (V): ");
    Serial.println(_fc->state.voltage, 2);
    Serial.print("Current Draw (A): ");
    Serial.println(_fc->state.current, 2);
    Serial.print("Free Heap (bytes): ");
    Serial.println(esp_get_free_heap_size());
    Serial.print("Min Free Heap (bytes): ");
    Serial.println(esp_get_minimum_free_heap_size());
    Serial.println("---------------------");
}

void CommunicationManager::_handleVersionCommand() const
{
    if (_currentMode == OperatingMode::API)
    {
        StaticJsonDocument<JSON_DOC_SMALL_SIZE> doc;
        doc["version"] = FIRMWARE_VERSION;
        serializeJson(doc, Serial);
        Serial.println();
    }
    else
    {
        Serial.print("Firmware Version: ");
        Serial.println(FIRMWARE_VERSION);
    }
}