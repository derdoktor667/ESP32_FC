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
    IMU_PROTOCOL
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
    { "motor.idle_speed", SettingType::FLOAT, &settings.motorIdleSpeedPercent, 1.0f },
    { "motor.dshot_mode", SettingType::DSHOT_MODE, &settings.dshotMode, 1.0f },
};
const int numSettings = sizeof(settingsRegistry) / sizeof(Setting);

// Forward declarations for helpers
void _printGetResponse(const String& param, const String& value, bool isApiMode, bool isString = false);
void _printSetResponse(const String& param, const String& value, bool success, bool isApiMode, bool isString = false);

// --- Public Methods ---

CommunicationManager::CommunicationManager(FlightController* fc) : _fc(fc) {}

void CommunicationManager::begin() {}

void CommunicationManager::update(const FlightState &state) {
    _handleSerialInput();
    if (_currentMode == OperatingMode::API && millis() - _lastApiPingTime > API_MODE_TIMEOUT_MS) {
        Serial.println("{\"status\":\"api_mode_timeout\"}");
        _currentMode = OperatingMode::FLIGHT;
        settings.enableLogging = false;
    }
    if (_currentMode == OperatingMode::API && settings.enableLogging && millis() - _lastSerialLogTime >= settings.printIntervalMs) {
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
            if (input.equalsIgnoreCase("cli")) {
                _currentMode = OperatingMode::CLI;
                settings.enableLogging = false;
                Serial.println("--- CLI Activated ---");
                Serial.print("ESP32_FC > ");
            } else if (input.equalsIgnoreCase("api")) {
                _currentMode = OperatingMode::API;
                settings.enableLogging = true;
                Serial.println("{\"status\":\"api_mode_activated\"}");
            }
            break;
        case OperatingMode::CLI: {
            String commandName = (input.indexOf(' ') != -1) ? input.substring(0, input.indexOf(' ')) : input;
            if (commandName.equalsIgnoreCase("exit")) {
                _currentMode = OperatingMode::FLIGHT;
                Serial.println("--- CLI Deactivated ---");
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
    } else if (commandName.equals("ping") && isApiMode) {
        _lastApiPingTime = millis();
    } else if (commandName.equals("help") && !isApiMode) {
        _printCliHelp();
    } else {
        if (isApiMode) Serial.println("{\"error\":\"Unknown command\"}");
        else { Serial.print("Unknown command: "); Serial.println(commandName); }
    }
}

// --- Refactored Command Implementations ---

void CommunicationManager::_printCliHelp() {
    // This can also be refactored to use the registry
    Serial.println("Help not implemented yet.");
}

void CommunicationManager::_handleDumpCommand() {
    // This can also be refactored to use the registry
    Serial.println("Dump not implemented yet.");
}

void CommunicationManager::_handleDumpJsonCommand() {
    _lastApiPingTime = millis();
    String json = "{\"settings\":{";
    for (int i = 0; i < numSettings; ++i) {
        const Setting& s = settingsRegistry[i];
        if (i > 0) json += ",";
        json += "\"" + String(s.name) + "\":" ;
        switch (s.type) {
            case SettingType::FLOAT: json += String(*(float*)s.value / s.scaleFactor, 4); break;
            case SettingType::UINT16: json += String(*(uint16_t*)s.value); break;
            case SettingType::RECEIVER_PROTOCOL: json += String(*(uint8_t*)s.value); break;
            case SettingType::IMU_PROTOCOL: json += String(*(uint8_t*)s.value); break;
            case SettingType::DSHOT_MODE: json += "\"" + getDShotModeString(*(dshot_mode_t*)s.value) + "\""; break;
        }
    }
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
        String key = "rx.map.";
        String inputName = getFlightControlInputString((FlightControlInput)i);
        inputName.toLowerCase();
        key += inputName;
        json += ",\"" + key + "\":" + String(settings.channelMapping.channel[i]);
    }
    json += "}}";
    Serial.println(json);
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
        if (isApiMode) Serial.println("{\"error\":\"Invalid 'set' format\"}");
        else Serial.println("Invalid 'set' format. Use: set <parameter> <value>");
        return;
    }
    String param = args.substring(0, lastSpace);
    String valueStr = args.substring(lastSpace + 1);

    for (int i = 0; i < numSettings; ++i) {
        const Setting& s = settingsRegistry[i];
        if (param.equalsIgnoreCase(s.name)) {
            bool success = true;
            switch (s.type) {
                case SettingType::FLOAT: *(float*)s.value = valueStr.toFloat() * s.scaleFactor; break;
                case SettingType::UINT16: *(uint16_t*)s.value = (uint16_t)valueStr.toInt(); break;
                case SettingType::RECEIVER_PROTOCOL: {
                    int p = valueStr.toInt();
                    if (p >= 0 && p < RECEIVER_PROTOCOL_COUNT) *(ReceiverProtocol*)s.value = (ReceiverProtocol)p; else success = false;
                    break;
                }
                case SettingType::IMU_PROTOCOL: {
                    int p = valueStr.toInt();
                    if (p >= 0 && p < IMU_PROTOCOL_COUNT) *(ImuProtocol*)s.value = (ImuProtocol)p; else success = false;
                    break;
                }
                case SettingType::DSHOT_MODE: {
                    if (valueStr.equalsIgnoreCase("DSHOT_OFF")) *(dshot_mode_t*)s.value = DSHOT_OFF;
                    else if (valueStr.equalsIgnoreCase("DSHOT150")) *(dshot_mode_t*)s.value = DSHOT150;
                    else if (valueStr.equalsIgnoreCase("DSHOT300")) *(dshot_mode_t*)s.value = DSHOT300;
                    else if (valueStr.equalsIgnoreCase("DSHOT600")) *(dshot_mode_t*)s.value = DSHOT600;
                    else if (valueStr.equalsIgnoreCase("DSHOT1200")) *(dshot_mode_t*)s.value = DSHOT1200;
                    else success = false;
                    break;
                }
            }
            _printSetResponse(param, valueStr, success, isApiMode, (s.type == SettingType::DSHOT_MODE));
            return;
        }
    }
    if (param.startsWith("rx.map.")) {
        String inputName = param.substring(7);
        int channelValue = valueStr.toInt();
        bool mapping_found = false;
        if (channelValue >= 0 && channelValue < RECEIVER_CHANNEL_COUNT) {
            for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
                if (inputName.equalsIgnoreCase(getFlightControlInputString((FlightControlInput)i))) {
                    settings.channelMapping.channel[i] = channelValue;
                    mapping_found = true;
                    break;
                }
            }
        }
        _printSetResponse(param, valueStr, mapping_found, isApiMode, false);
        return;
    }
    _printSetResponse(param, valueStr, false, isApiMode, false);
}


void CommunicationManager::_printFlightStatus(const FlightState &state) {
    if (isnan(state.attitude.roll) || isnan(state.attitude.pitch) || isnan(state.attitude.yaw)) {
        Serial.println("{\"error\":\"Attitude data is NaN. Check IMU connection.\"}");
        return;
    }
    Serial.print("{\"live_data\":{");
    Serial.print("\"attitude\":{");
    Serial.print("\"roll\":"); Serial.print(state.attitude.roll, 2);
    Serial.print(",\"pitch\":"); Serial.print(state.attitude.pitch, 2);
    Serial.print(",\"yaw\":"); Serial.print(state.attitude.yaw, 2);
    Serial.print("}, ");
    Serial.print("\"status\":{");
    Serial.print("\"armed\":"); Serial.print(state.isArmed);
    Serial.print(",\"failsafe\":"); Serial.print(state.isFailsafeActive);
    Serial.print(",\"mode\":\"");
    switch (state.currentFlightMode) {
        case ACRO_MODE: Serial.print("ACRO"); break;
        case ANGLE_MODE: Serial.print("ANGLE"); break;
        default: Serial.print("UNKNOWN"); break;
    }
    Serial.print("\"}, ");
    Serial.print("\"motor_output\":[");
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print(state.motorOutputs[i]);
        if (i < NUM_MOTORS - 1) Serial.print(",");
    }
    Serial.print("]");
    Serial.println("}}");
}

// --- Command Helpers for Get/Set ---

void CommunicationManager::_printGetResponse(const String& param, const String& value, bool isApiMode, bool isString) {
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

void CommunicationManager::_printSetResponse(const String& param, const String& value, bool success, bool isApiMode, bool isString) {
    if (isApiMode) {
        if (success) {
            Serial.print("{\"set\":{\"");
            Serial.print(param);
            Serial.print("\":");
            if (isString) Serial.print("\"");
            Serial.print(value);
            if (isString) Serial.print("\"");
            Serial.println(",\"status\":\"success\"}}");
        } else {
            Serial.print("{\"error\":\"Invalid value for parameter: ");
            Serial.print(param);
            Serial.println("\"}");
        }
    } else {
        if (success) {
            Serial.print("Set ");
            Serial.print(param);
            Serial.print(" to ");
            Serial.println(value);
        } else {
            Serial.println("Invalid value or parameter for 'set'.");
        }
    }
}