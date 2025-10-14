#include "CommunicationManager.h"
#include "flight_controller.h"      // Include full definition of FlightController
#include "../config/config.h"       // Include for NUM_CHANNELS
#include "../config/settings.h"
#include "../config/FlightState.h"
#include <Arduino.h>

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

// --- Public Methods ---

CommunicationManager::CommunicationManager(FlightController* fc) : _fc(fc) {}

void CommunicationManager::begin() {}

void CommunicationManager::update(const FlightState &state)
{
    _handleSerialInput();

    // Check for API mode timeout
    if (_currentMode == OperatingMode::API && millis() - _lastApiPingTime > API_MODE_TIMEOUT_MS)
    {
        Serial.println("{\"status\":\"api_mode_timeout\"}");
        _currentMode = OperatingMode::FLIGHT;
        settings.enableLogging = false; // Re-enable the flight logic
    }

    // Stream live data if in API mode and logging is enabled
    if (_currentMode == OperatingMode::API && settings.enableLogging && millis() - _lastSerialLogTime >= settings.printIntervalMs)
    {
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
        _lastApiPingTime = millis(); // Reset the ping timer
    } else if (commandName.equals("help") && !isApiMode) {
        _printCliHelp();
    } else {
        if (isApiMode) Serial.println("{\"error\":\"Unknown command\"}");
        else {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        }
    }
}

// --- Private Methods: Command Implementations ---

void CommunicationManager::_printCliHelp() {
    Serial.println("\n--- Flight Controller CLI Help ---");
    Serial.println("\n[ General Commands ]");
    Serial.printf("  %-22s - %s\n", "help", "Display this help message.");
    Serial.printf("  %-22s - %s\n", "exit", "Deactivate CLI and return to flight mode.");
    Serial.printf("  %-22s - %s\n", "reboot", "Reboot the ESP32 flight controller.");

    Serial.println("\n[ Settings Management ]");
    Serial.printf("  %-22s - %s\n", "get <parameter>", "Get a specific setting or a whole category.");
    Serial.printf("  %-22s - %s\n", "set <param> <value>", "Set a new value for a parameter.");
    Serial.printf("  %-22s - %s\n", "dump", "Display all current settings.");
    Serial.printf("  %-22s - %s\n", "save", "Save all settings to flash and reboot.");
    Serial.printf("  %-22s - %s\n", "reset", "Reset all settings to defaults, save, and reboot.");

    Serial.println("\n[ Calibration Commands ]");
    Serial.printf("  %-22s - %s\n", "calibrate_imu", "Initiate IMU sensor calibration.");

    Serial.println("\n[ Available Settings ]");
    Serial.println("  You can 'get' a whole category (e.g., 'get pid') or get/set a specific parameter.");

    Serial.println("\n  --- PID Settings ---");
    Serial.println("    pid.roll.kp, pid.roll.ki, pid.roll.kd");
    Serial.println("    pid.pitch.kp, pid.pitch.ki, pid.pitch.kd");
    Serial.println("    pid.yaw.kp, pid.yaw.ki, pid.yaw.kd");
    Serial.println("    pid.integral_limit");

    Serial.println("\n  --- Rate Settings ---");
    Serial.println("    rates.angle       (Max angle in ANGLE mode)");
    Serial.println("    rates.yaw         (Max yaw rate in deg/s)");
    Serial.println("    rates.acro        (Max roll/pitch rate in deg/s for ACRO mode)");

    Serial.println("\n  --- Filter Settings ---");
    Serial.println("    madgwick.sample_freq");
    Serial.println("    madgwick.beta");

    Serial.println("\n  --- Receiver Settings ---");
    Serial.println("    rx.min, rx.max");
    Serial.println("    rx.arming_threshold, rx.failsafe_threshold");
    Serial.println("    rx.protocol (0=IBUS, 1=PPM)");
    Serial.println("    rx.map.throttle, rx.map.roll, rx.map.pitch, rx.map.yaw");
    Serial.println("    rx.map.arm_switch, rx.map.failsafe_switch, rx.map.flight_mode_switch");


    Serial.println("\n  --- IMU Settings ---");
    Serial.println("    imu.protocol (0=MPU6050)");

    Serial.println("\n  --- Motor Settings ---");
    Serial.println("    motor.idle_speed");
    Serial.println("    motor.dshot_mode (DSHOT_OFF, DSHOT150, DSHOT300, DSHOT600, DSHOT1200)");


    Serial.println("\n--- End of Help ---");
}

void CommunicationManager::_printPidSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"pid\":{");
        Serial.print("\"roll\":{\"kp\":"); Serial.print(settings.pidRoll.kp / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"ki\":"); Serial.print(settings.pidRoll.ki / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"kd\":"); Serial.print(settings.pidRoll.kd / (float)PID_SCALE_FACTOR, 3);
        Serial.print("},");
        Serial.print("\"pitch\":{\"kp\":"); Serial.print(settings.pidPitch.kp / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"ki\":"); Serial.print(settings.pidPitch.ki / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"kd\":"); Serial.print(settings.pidPitch.kd / (float)PID_SCALE_FACTOR, 3);
        Serial.print("},");
        Serial.print("\"yaw\":{\"kp\":"); Serial.print(settings.pidYaw.kp / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"ki\":"); Serial.print(settings.pidYaw.ki / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"kd\":"); Serial.print(settings.pidYaw.kd / (float)PID_SCALE_FACTOR, 3);
        Serial.print("},");
        Serial.print("\"integral_limit\":"); Serial.print(settings.pidIntegralLimit, 4);
        Serial.println("}}");
    } else {
        Serial.println("--- PID Settings ---");
        Serial.printf("  %-25s: %.3f\n", "pid.roll.kp", settings.pidRoll.kp / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.roll.ki", settings.pidRoll.ki / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.roll.kd", settings.pidRoll.kd / (float)PID_SCALE_FACTOR);
        Serial.println();
        Serial.printf("  %-25s: %.3f\n", "pid.pitch.kp", settings.pidPitch.kp / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.pitch.ki", settings.pidPitch.ki / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.pitch.kd", settings.pidPitch.kd / (float)PID_SCALE_FACTOR);
        Serial.println();
        Serial.printf("  %-25s: %.3f\n", "pid.yaw.kp", settings.pidYaw.kp / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.yaw.ki", settings.pidYaw.ki / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.yaw.kd", settings.pidYaw.kd / (float)PID_SCALE_FACTOR);
        Serial.println();
        Serial.printf("  %-25s: %.2f\n", "pid.integral_limit", settings.pidIntegralLimit);
    }
}

void CommunicationManager::_printRatesSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"rates\":{");
        Serial.print("\"angle\":"); Serial.print(settings.rates.maxAngleRollPitch, 4);
        Serial.print(",\"yaw\":"); Serial.print(settings.rates.maxRateYaw, 4);
        Serial.print(",\"acro\":"); Serial.print(settings.rates.maxRateRollPitch, 4);
        Serial.println("}}");
    } else {
        Serial.println("\n--- Rate Settings ---");
        Serial.printf("  %-25s: %.2f (deg)\n", "rates.angle", settings.rates.maxAngleRollPitch);
        Serial.printf("  %-25s: %.2f (deg/s)\n", "rates.yaw", settings.rates.maxRateYaw);
        Serial.printf("  %-25s: %.2f (deg/s)\n", "rates.acro", settings.rates.maxRateRollPitch);
    }
}

void CommunicationManager::_printFilterSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"filter\":{");
        Serial.print("\"madgwick.sample_freq\":"); Serial.print(settings.filter.madgwickSampleFreq, 1);
        Serial.print(",\"madgwick.beta\":"); Serial.print(settings.filter.madgwickBeta, 4);
        Serial.println("}}");
    } else {
        Serial.println("\n--- Filter Settings ---");
        Serial.printf("  %-25s: %.1f (Hz)\n", "madgwick.sample_freq", settings.filter.madgwickSampleFreq);
        Serial.printf("  %-25s: %.4f\n", "madgwick.beta", settings.filter.madgwickBeta);
    }
}

void CommunicationManager::_printReceiverSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"receiver\":{");
        Serial.print("\"min\":"); Serial.print(settings.receiver.ibusMinValue);
        Serial.print(",\"max\":"); Serial.print(settings.receiver.ibusMaxValue);
        Serial.print(",\"arming_threshold\":"); Serial.print(settings.receiver.armingThreshold);
        Serial.print(",\"failsafe_threshold\":"); Serial.print(settings.receiver.failsafeThreshold);
        Serial.print(",\"protocol\":"); Serial.print((int)settings.receiverProtocol);
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
            String key = "map.";
            String inputName = getFlightControlInputString((FlightControlInput)i);
            inputName.toLowerCase();
            key += inputName;
            Serial.print(",\""); Serial.print(key); Serial.print("\":"); Serial.print(settings.channelMapping.channel[i]);
        }
        Serial.println("}}");
    } else {
        Serial.println("\n--- Receiver Settings ---");
        Serial.printf("  %-25s: %d\n", "rx.min", settings.receiver.ibusMinValue);
        Serial.printf("  %-25s: %d\n", "rx.max", settings.receiver.ibusMaxValue);
        Serial.printf("  %-25s: %d\n", "rx.arming_threshold", settings.receiver.armingThreshold);
        Serial.printf("  %-25s: %d\n", "rx.failsafe_threshold", settings.receiver.failsafeThreshold);
        Serial.printf("  %-25s: %s\n", "rx.protocol", getReceiverProtocolString(settings.receiverProtocol).c_str());
        Serial.println("\n--- Receiver Channel Mapping ---");
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
            Serial.printf("  %-25s: %d\n", getFlightControlInputString((FlightControlInput)i).c_str(), settings.channelMapping.channel[i]);
        }
    }
}

void CommunicationManager::_printImuSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"imu\":{");
        Serial.print("\"protocol\":"); Serial.print((int)settings.imuProtocol);
        Serial.println("}}");
    } else {
        Serial.println("\n--- IMU Settings ---");
        Serial.printf("  %-25s: %s\n", "imu.protocol", getImuProtocolString(settings.imuProtocol).c_str());
    }
}

void CommunicationManager::_printMotorSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"motor\":{");
        Serial.print("\"idle_speed\":"); Serial.print(settings.motorIdleSpeedPercent, 1);
        Serial.print(",\"dshot_mode\":\""); Serial.print(getDShotModeString(settings.dshotMode)); Serial.print("\"");
        Serial.println("}}");
    } else {
        Serial.println("\n--- Motor Settings ---");
        Serial.printf("  %-25s: %.1f (%%)\n", "motor.idle_speed", settings.motorIdleSpeedPercent);
        Serial.printf("  %-25s: %s\n", "motor.dshot_mode", getDShotModeString(settings.dshotMode).c_str());
    }
}

void CommunicationManager::_handleGetCommand(String args, bool isApiMode) {
    if (args.startsWith("pid")) {
        _handleGetPid(args, isApiMode);
    } else if (args.startsWith("rates")) {
        _handleGetRates(args, isApiMode);
    } else if (args.startsWith("filter") || args.startsWith("madgwick")) { // Allow both for convenience
        _handleGetFilter(args, isApiMode);
    } else if (args.startsWith("rx")) {
        _handleGetReceiver(args, isApiMode);
    } else if (args.startsWith("imu")) {
        _handleGetImu(args, isApiMode);
    } else if (args.startsWith("motor")) {
        _handleGetMotor(args, isApiMode);
    } else {
        if (isApiMode) {
            Serial.print("{\"error\":\"Unknown parameter for 'get': ");
            Serial.print(args);
            Serial.println("\"}");
        } else {
            Serial.println("Unknown parameter for 'get'.");
        }
    }
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

    if (param.startsWith("pid")) {
        _handleSetPid(param, valueStr, isApiMode);
    } else if (param.startsWith("rates")) {
        _handleSetRates(param, valueStr, isApiMode);
    } else if (param.startsWith("madgwick")) {
        _handleSetFilter(param, valueStr, isApiMode);
    } else if (param.startsWith("rx")) {
        _handleSetReceiver(param, valueStr, isApiMode);
    } else if (param.startsWith("imu")) {
        _handleSetImu(param, valueStr, isApiMode);
    } else if (param.startsWith("motor")) {
        _handleSetMotor(param, valueStr, isApiMode);
    } else {
        if (isApiMode) {
            Serial.print("{\"error\":\"Invalid value for parameter: ");
            Serial.print(param);
            Serial.println("\"}");
        } else {
            Serial.println("Invalid value or parameter for 'set'.");
        }
    }
}

void CommunicationManager::_handleDumpCommand() {
    Serial.println("\n--- [ Flight Controller Settings ] ---\n");
    _printPidSettings(false);
    _printRatesSettings(false);
    _printFilterSettings(false);
    _printReceiverSettings(false);
    _printImuSettings(false);
    _printMotorSettings(false);
    Serial.println("\n--------------------------------------");
}

void CommunicationManager::_handleDumpJsonCommand() {
    Serial.print("{\"settings\":{");
    Serial.print("\"pid.roll.kp\":"); Serial.print(settings.pidRoll.kp / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.roll.ki\":"); Serial.print(settings.pidRoll.ki / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.roll.kd\":"); Serial.print(settings.pidRoll.kd / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.pitch.kp\":"); Serial.print(settings.pidPitch.kp / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.pitch.ki\":"); Serial.print(settings.pidPitch.ki / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.pitch.kd\":"); Serial.print(settings.pidPitch.kd / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.yaw.kp\":"); Serial.print(settings.pidYaw.kp / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.yaw.ki\":"); Serial.print(settings.pidYaw.ki / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.yaw.kd\":"); Serial.print(settings.pidYaw.kd / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.integral_limit\":"); Serial.print(settings.pidIntegralLimit, 4);
    Serial.print(",\"rates.angle\":"); Serial.print(settings.rates.maxAngleRollPitch, 4);
    Serial.print(",\"rates.yaw\":"); Serial.print(settings.rates.maxRateYaw, 4);
    Serial.print(",\"rates.acro\":"); Serial.print(settings.rates.maxRateRollPitch, 4);
    Serial.print(",\"madgwick.sample_freq\":"); Serial.print(settings.filter.madgwickSampleFreq, 1);
    Serial.print(",\"madgwick.beta\":"); Serial.print(settings.filter.madgwickBeta, 4);
    Serial.print(",\"rx.min\":"); Serial.print(settings.receiver.ibusMinValue);
    Serial.print(",\"rx.max\":"); Serial.print(settings.receiver.ibusMaxValue);
    Serial.print(",\"rx.arming_threshold\":"); Serial.print(settings.receiver.armingThreshold);
    Serial.print(",\"rx.failsafe_threshold\":"); Serial.print(settings.receiver.failsafeThreshold);
    Serial.print(",\"rx.protocol\":"); Serial.print((int)settings.receiverProtocol);
    Serial.print(",\"imu.protocol\":"); Serial.print((int)settings.imuProtocol);
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
        String key = "rx.map.";
        String inputName = getFlightControlInputString((FlightControlInput)i);
        inputName.toLowerCase();
        key += inputName;
        Serial.print(",\""); Serial.print(key); Serial.print("\":"); Serial.print(settings.channelMapping.channel[i]);
    }
    Serial.print(",\"motor.idle_speed\":"); Serial.print(settings.motorIdleSpeedPercent, 1);
    Serial.print(",\"motor.dshot_mode\":\""); Serial.print(getDShotModeString(settings.dshotMode)); Serial.print("\"");
    Serial.println("}}");
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
    Serial.print("},");

    Serial.print("\"status\":{");
    Serial.print("\"armed\":"); Serial.print(state.isArmed);
    Serial.print(",\"failsafe\":"); Serial.print(state.isFailsafeActive);
    Serial.print(",\"mode\":\"");
    switch (state.currentFlightMode) {
        case ACRO_MODE: Serial.print("ACRO"); break;
        case ANGLE_MODE: Serial.print("ANGLE"); break;
        default: Serial.print("UNKNOWN"); break;
    }
    Serial.print("\"},");

    Serial.print("\"motor_output\":[");
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print(state.motorOutputs[i]);
        if (i < NUM_MOTORS - 1) Serial.print(",");
    }
    Serial.print("]");

    Serial.println("}}");
}

// --- Command Helpers for Get/Set ---

// Helper to format and print the response for a 'get' command.
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

// Helper to format and print the response for a 'set' command.
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

void CommunicationManager::_handleGetPid(String args, bool isApiMode) {
    if (args.equals("pid")) {
        _printPidSettings(isApiMode);
        return;
    }
    String valueStr = "";
    bool found = true;
    if (args.equals("pid.roll.kp")) valueStr = String(settings.pidRoll.kp / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.roll.ki")) valueStr = String(settings.pidRoll.ki / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.roll.kd")) valueStr = String(settings.pidRoll.kd / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.pitch.kp")) valueStr = String(settings.pidPitch.kp / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.pitch.ki")) valueStr = String(settings.pidPitch.ki / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.pitch.kd")) valueStr = String(settings.pidPitch.kd / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.yaw.kp")) valueStr = String(settings.pidYaw.kp / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.yaw.ki")) valueStr = String(settings.pidYaw.ki / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.yaw.kd")) valueStr = String(settings.pidYaw.kd / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.integral_limit")) valueStr = String(settings.pidIntegralLimit, 4);
    else found = false;

    if (found) _printGetResponse(args, valueStr, isApiMode);
}

void CommunicationManager::_handleGetRates(String args, bool isApiMode) {
    if (args.equals("rates")) {
        _printRatesSettings(isApiMode);
        return;
    }
    String valueStr = "";
    bool found = true;
    if (args.equals("rates.angle")) valueStr = String(settings.rates.maxAngleRollPitch, 4);
    else if (args.equals("rates.yaw")) valueStr = String(settings.rates.maxRateYaw, 4);
    else if (args.equals("rates.acro")) valueStr = String(settings.rates.maxRateRollPitch, 4);
    else found = false;

    if (found) _printGetResponse(args, valueStr, isApiMode);
}

void CommunicationManager::_handleGetFilter(String args, bool isApiMode) {
    if (args.equals("filter")) {
        _printFilterSettings(isApiMode);
        return;
    }
    String valueStr = "";
    bool found = true;
    if (args.equals("madgwick.sample_freq")) valueStr = String(settings.filter.madgwickSampleFreq, 1);
    else if (args.equals("madgwick.beta")) valueStr = String(settings.filter.madgwickBeta, 4);
    else found = false;

    if (found) _printGetResponse(args, valueStr, isApiMode);
}

void CommunicationManager::_handleGetReceiver(String args, bool isApiMode) {
    if (args.equals("receiver")) {
        _printReceiverSettings(isApiMode);
        return;
    }
    String valueStr = "";
    bool found = true;
    if (args.equals("rx.min")) valueStr = String(settings.receiver.ibusMinValue);
    else if (args.equals("rx.max")) valueStr = String(settings.receiver.ibusMaxValue);
    else if (args.equals("rx.arming_threshold")) valueStr = String(settings.receiver.armingThreshold);
    else if (args.equals("rx.failsafe_threshold")) valueStr = String(settings.receiver.failsafeThreshold);
    else if (args.equals("rx.protocol")) valueStr = String((int)settings.receiverProtocol);
    else if (args.startsWith("rx.map.")) {
        String inputName = args.substring(RX_MAP_PREFIX_LENGTH);
        found = false;
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
            if (inputName.equalsIgnoreCase(getFlightControlInputString((FlightControlInput)i))) {
                valueStr = String(settings.channelMapping.channel[i]);
                found = true;
                break;
            }
        }
    } else if (args.equals("rx.channels")) {
        if (isApiMode) {
            Serial.print("{\"rx.channels\":[");
            for (int i = 0; i < RECEIVER_CHANNEL_COUNT; ++i) {
                Serial.print(_fc->state.receiverChannels[i]);
                if (i < RECEIVER_CHANNEL_COUNT - 1) Serial.print(",");
            }
            Serial.println("]}");
        } else {
            Serial.print("Receiver Channels: ");
            for (int i = 0; i < RECEIVER_CHANNEL_COUNT; ++i) {
                Serial.print(_fc->state.receiverChannels[i]);
                Serial.print(" ");
            }
            Serial.println();
        }
        return; // Special case, doesn't use the standard response format
    }
    else found = false;

    if (found) _printGetResponse(args, valueStr, isApiMode);
}

void CommunicationManager::_handleGetImu(String args, bool isApiMode) {
    if (args.equals("imu")) {
        _printImuSettings(isApiMode);
        return;
    }
    String valueStr = "";
    bool found = true;
    if (args.equals("imu.protocol")) valueStr = String((int)settings.imuProtocol);
    else found = false;

    if (found) _printGetResponse(args, valueStr, isApiMode);
}

void CommunicationManager::_handleGetMotor(String args, bool isApiMode) {
    if (args.equals("motor")) {
        _printMotorSettings(isApiMode);
        return;
    }
    String valueStr = "";
    bool found = true;
    bool isString = false;
    if (args.equals("motor.idle_speed")) valueStr = String(settings.motorIdleSpeedPercent, 1);
    else if (args.equals("motor.dshot_mode")) {
        valueStr = getDShotModeString(settings.dshotMode);
        isString = true;
    }
    else found = false;

    if (found) _printGetResponse(args, valueStr, isApiMode, isString);
}

void CommunicationManager::_handleSetPid(String param, String value, bool isApiMode) {
    bool success = true;
    if (param.equals("pid.roll.kp")) settings.pidRoll.kp = round(value.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.roll.ki")) settings.pidRoll.ki = round(value.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.roll.kd")) settings.pidRoll.kd = round(value.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.kp")) settings.pidPitch.kp = round(value.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.ki")) settings.pidPitch.ki = round(value.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.kd")) settings.pidPitch.kd = round(value.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.kp")) settings.pidYaw.kp = round(value.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.ki")) settings.pidYaw.ki = round(value.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.kd")) settings.pidYaw.kd = round(value.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.integral_limit")) settings.pidIntegralLimit = value.toFloat();
    else success = false;

    _printSetResponse(param, value, success, isApiMode);
}

void CommunicationManager::_handleSetRates(String param, String value, bool isApiMode) {
    bool success = true;
    if (param.equals("rates.angle")) settings.rates.maxAngleRollPitch = value.toFloat();
    else if (param.equals("rates.yaw")) settings.rates.maxRateYaw = value.toFloat();
    else if (param.equals("rates.acro")) settings.rates.maxRateRollPitch = value.toFloat();
    else success = false;

    _printSetResponse(param, value, success, isApiMode);
}

void CommunicationManager::_handleSetFilter(String param, String value, bool isApiMode) {
    bool success = true;
    if (param.equals("madgwick.sample_freq")) settings.filter.madgwickSampleFreq = value.toFloat();
    else if (param.equals("madgwick.beta")) settings.filter.madgwickBeta = value.toFloat();
    else success = false;

    _printSetResponse(param, value, success, isApiMode);
}

void CommunicationManager::_handleSetReceiver(String param, String value, bool isApiMode) {
    bool success = true;
    if (param.equals("rx.min")) settings.receiver.ibusMinValue = value.toInt();
    else if (param.equals("rx.max")) settings.receiver.ibusMaxValue = value.toInt();
    else if (param.equals("rx.arming_threshold")) settings.receiver.armingThreshold = value.toInt();
    else if (param.equals("rx.failsafe_threshold")) settings.receiver.failsafeThreshold = value.toInt();
    else if (param.equals("rx.protocol")) {
        int p = value.toInt();
        if (p >= 0 && p < ReceiverProtocol::RECEIVER_PROTOCOL_COUNT) settings.receiverProtocol = (ReceiverProtocol)p; else success = false;
    } else if (param.startsWith("rx.map.")) {
        String inputName = param.substring(RX_MAP_PREFIX_LENGTH);
        int channelValue = value.toInt();
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
        if (!mapping_found) success = false;
    }
    else success = false;

    _printSetResponse(param, value, success, isApiMode);
}

void CommunicationManager::_handleSetImu(String param, String value, bool isApiMode) {
    bool success = true;
    if (param.equals("imu.protocol")) {
        int p = value.toInt();
        if (p >= 0 && p < ImuProtocol::IMU_PROTOCOL_COUNT) settings.imuProtocol = (ImuProtocol)p; else success = false;
    } else {
        success = false;
    }
    _printSetResponse(param, value, success, isApiMode);
}

void CommunicationManager::_handleSetMotor(String param, String value, bool isApiMode) {
    bool success = true;
    bool isString = false;
    if (param.equals("motor.idle_speed")) {
        settings.motorIdleSpeedPercent = round(value.toFloat() * 10.0f) / 10.0f;
    } else if (param.equals("motor.dshot_mode")) {
        isString = true;
        if (value.equalsIgnoreCase("DSHOT_OFF")) settings.dshotMode = DSHOT_OFF;
        else if (value.equalsIgnoreCase("DSHOT150")) settings.dshotMode = DSHOT150;
        else if (value.equalsIgnoreCase("DSHOT300")) settings.dshotMode = DSHOT300;
        else if (value.equalsIgnoreCase("DSHOT600")) settings.dshotMode = DSHOT600;
        else if (value.equalsIgnoreCase("DSHOT1200")) settings.dshotMode = DSHOT1200;
        else success = false;
    } else {
        success = false;
    }
    _printSetResponse(param, value, success, isApiMode, isString);
}