#include "CliCommandProcessor.h"
#include "src/main/flight_controller.h"

// --- Helper Functions for String Conversion (defined here for now) ---
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

CliCommandProcessor::CliCommandProcessor(FlightController* fc, FlightControllerSettings& settings)
    : _fc(fc), _settings(settings) {}

void CliCommandProcessor::handleGetCommand(String args, bool isApiMode) {
    // This function remains complex due to the number of settings.
    // The logic is straightforward mapping of string to value.
    String param = args;
    String valueStr = "";
    bool found = true;

    if (param.equals("pid")) {
        _printPidSettings(isApiMode);
        return;
    } else if (param.equals("rates")) {
        _printRatesSettings(isApiMode);
        return;
    } else if (param.equals("filter")) {
        _printFilterSettings(isApiMode);
        return;
    } else if (param.equals("receiver")) {
        _printReceiverSettings(isApiMode);
        return;
    } else if (param.equals("imu")) {
        _printImuSettings(isApiMode);
        return;
    } else if (param.equals("motor")) {
        _printMotorSettings(isApiMode);
        return;
    }

    if (param.equals("pid.roll.kp")) valueStr = String(_settings.pidRoll.kp / (float)PID_SCALE_FACTOR, 3);
    else if (param.equals("pid.roll.ki")) valueStr = String(_settings.pidRoll.ki / (float)PID_SCALE_FACTOR, 3);
    else if (param.equals("pid.roll.kd")) valueStr = String(_settings.pidRoll.kd / (float)PID_SCALE_FACTOR, 3);
    else if (param.equals("pid.pitch.kp")) valueStr = String(_settings.pidPitch.kp / (float)PID_SCALE_FACTOR, 3);
    else if (param.equals("pid.pitch.ki")) valueStr = String(_settings.pidPitch.ki / (float)PID_SCALE_FACTOR, 3);
    else if (param.equals("pid.pitch.kd")) valueStr = String(_settings.pidPitch.kd / (float)PID_SCALE_FACTOR, 3);
    else if (param.equals("pid.yaw.kp")) valueStr = String(_settings.pidYaw.kp / (float)PID_SCALE_FACTOR, 3);
    else if (param.equals("pid.yaw.ki")) valueStr = String(_settings.pidYaw.ki / (float)PID_SCALE_FACTOR, 3);
    else if (param.equals("pid.yaw.kd")) valueStr = String(_settings.pidYaw.kd / (float)PID_SCALE_FACTOR, 3);
    else if (param.equals("pid.integral_limit")) valueStr = String(_settings.pidIntegralLimit, 4);
    else if (param.equals("rates.angle")) valueStr = String(_settings.rates.maxAngleRollPitch, 4);
    else if (param.equals("rates.yaw")) valueStr = String(_settings.rates.maxRateYaw, 4);
    else if (param.equals("rates.acro")) valueStr = String(_settings.rates.maxRateRollPitch, 4);
    else if (param.equals("madgwick.sample_freq")) valueStr = String(_settings.filter.madgwickSampleFreq, 1);
    else if (param.equals("madgwick.beta")) valueStr = String(_settings.filter.madgwickBeta, 4);
    else if (param.equals("rx.min")) valueStr = String(_settings.receiver.ibusMinValue);
    else if (param.equals("rx.max")) valueStr = String(_settings.receiver.ibusMaxValue);
    else if (param.equals("rx.arming_threshold")) valueStr = String(_settings.receiver.armingThreshold);
    else if (param.equals("rx.failsafe_threshold")) valueStr = String(_settings.receiver.failsafeThreshold);
    else if (param.equals("rx.protocol")) valueStr = String((int)_settings.receiverProtocol);
    else if (param.equals("imu.protocol")) valueStr = String((int)_settings.imuProtocol);
    else if (param.startsWith("rx.map.")) {
        String inputName = param.substring(7);
        found = false;
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
            if (inputName.equalsIgnoreCase(getFlightControlInputString((FlightControlInput)i))) {
                valueStr = String(_settings.channelMapping.channel[i]);
                found = true;
                break;
            }
        }
    } else if (param.equals("motor.idle_speed")) valueStr = String(_settings.motorIdleSpeedPercent, 1);
    else if (param.equals("motor.dshot_mode")) valueStr = String(getDShotModeString(_settings.dshotMode));
    else if (param.equals("rx.channels")) {
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
        return;
    }
    else found = false;

    if (isApiMode) {
        if (found) {
            Serial.print("{\"get\":{\"");
            Serial.print(param);
            Serial.print("\":");
            bool isString = param.equals("motor.dshot_mode");
            if (isString) Serial.print("\"");
            Serial.print(valueStr);
            if (isString) Serial.print("\"");
            Serial.println("}}");
        } else {
            Serial.print("{\"error\":\"Unknown parameter for 'get': ");
            Serial.print(param);
            Serial.println("\"}");
        }
    } else {
        if (found) Serial.println(valueStr);
        else Serial.println("Unknown parameter for 'get'.");
    }
}

void CliCommandProcessor::handleSetCommand(String args, bool isApiMode) {
    // This function is also complex but necessary.
    int lastSpace = args.lastIndexOf(' ');
    if (lastSpace == -1) {
        if (isApiMode) Serial.println("{\"error\":\"Invalid 'set' format\"}");
        else Serial.println("Invalid 'set' format. Use: set <parameter> <value>");
        return;
    }

    String param = args.substring(0, lastSpace);
    String valueStr = args.substring(lastSpace + 1);
    bool success = true;

    if (param.equals("pid.roll.kp")) _settings.pidRoll.kp = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.roll.ki")) _settings.pidRoll.ki = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.roll.kd")) _settings.pidRoll.kd = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.kp")) _settings.pidPitch.kp = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.ki")) _settings.pidPitch.ki = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.kd")) _settings.pidPitch.kd = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.kp")) _settings.pidYaw.kp = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.ki")) _settings.pidYaw.ki = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.kd")) _settings.pidYaw.kd = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.integral_limit")) _settings.pidIntegralLimit = valueStr.toFloat();
    else if (param.equals("rates.angle")) _settings.rates.maxAngleRollPitch = valueStr.toFloat();
    else if (param.equals("rates.yaw")) _settings.rates.maxRateYaw = valueStr.toFloat();
    else if (param.equals("rates.acro")) _settings.rates.maxRateRollPitch = valueStr.toFloat();
    else if (param.equals("madgwick.sample_freq")) _settings.filter.madgwickSampleFreq = valueStr.toFloat();
    else if (param.equals("madgwick.beta")) _settings.filter.madgwickBeta = valueStr.toFloat();
    else if (param.equals("rx.min")) _settings.receiver.ibusMinValue = valueStr.toInt();
    else if (param.equals("rx.max")) _settings.receiver.ibusMaxValue = valueStr.toInt();
    else if (param.equals("rx.arming_threshold")) _settings.receiver.armingThreshold = valueStr.toInt();
    else if (param.equals("rx.failsafe_threshold")) _settings.receiver.failsafeThreshold = valueStr.toInt();
    else if (param.equals("rx.protocol")) {
        int p = valueStr.toInt();
        if (p >= 0 && p < RECEIVER_PROTOCOL_COUNT) _settings.receiverProtocol = (ReceiverProtocol)p; else success = false;
    } else if (param.equals("imu.protocol")) {
        int p = valueStr.toInt();
        if (p >= 0 && p < IMU_PROTOCOL_COUNT) _settings.imuProtocol = (ImuProtocol)p; else success = false;
    } else if (param.startsWith("rx.map.")) {
        String inputName = param.substring(7);
        int channelValue = valueStr.toInt();
        bool mapping_found = false;
        if (channelValue >= 0 && channelValue < RECEIVER_CHANNEL_COUNT) {
            for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
                if (inputName.equalsIgnoreCase(getFlightControlInputString((FlightControlInput)i))) {
                    _settings.channelMapping.channel[i] = channelValue;
                    mapping_found = true;
                    break;
                }
            }
        }
        if (!mapping_found) success = false;
    } else if (param.equals("motor.idle_speed")) {
        _settings.motorIdleSpeedPercent = round(valueStr.toFloat() * 10.0f) / 10.0f;
    } else if (param.equals("motor.dshot_mode")) {
        if (valueStr.equalsIgnoreCase("DSHOT_OFF")) _settings.dshotMode = DSHOT_OFF;
        else if (valueStr.equalsIgnoreCase("DSHOT150")) _settings.dshotMode = DSHOT150;
        else if (valueStr.equalsIgnoreCase("DSHOT300")) _settings.dshotMode = DSHOT300;
        else if (valueStr.equalsIgnoreCase("DSHOT600")) _settings.dshotMode = DSHOT600;
        else if (valueStr.equalsIgnoreCase("DSHOT1200")) _settings.dshotMode = DSHOT1200;
        else success = false;
    } else {
        success = false;
    }

    if (isApiMode) {
        if (success) {
            Serial.print("{\"set\":{\"");
            Serial.print(param);
            Serial.print("\":");
            bool isString = param.equals("motor.dshot_mode");
            if(isString) Serial.print("\"");
            Serial.print(valueStr);
            if(isString) Serial.print("\"");
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
            Serial.println(valueStr);
        } else {
            Serial.println("Invalid value or parameter for 'set'.");
        }
    }
}

void CliCommandProcessor::handleDumpCommand() {
    Serial.println("\n--- [ Flight Controller Settings ] ---\n");
    Serial.println("--- PID Settings ---");
    Serial.printf("  %-25s: %.3f\n", "pid.roll.kp", _settings.pidRoll.kp / (float)PID_SCALE_FACTOR);
    Serial.printf("  %-25s: %.3f\n", "pid.roll.ki", _settings.pidRoll.ki / (float)PID_SCALE_FACTOR);
    Serial.printf("  %-25s: %.3f\n", "pid.roll.kd", _settings.pidRoll.kd / (float)PID_SCALE_FACTOR);
    Serial.println();
    Serial.printf("  %-25s: %.3f\n", "pid.pitch.kp", _settings.pidPitch.kp / (float)PID_SCALE_FACTOR);
    Serial.printf("  %-25s: %.3f\n", "pid.pitch.ki", _settings.pidPitch.ki / (float)PID_SCALE_FACTOR);
    Serial.printf("  %-25s: %.3f\n", "pid.pitch.kd", _settings.pidPitch.kd / (float)PID_SCALE_FACTOR);
    Serial.println();
    Serial.printf("  %-25s: %.3f\n", "pid.yaw.kp", _settings.pidYaw.kp / (float)PID_SCALE_FACTOR);
    Serial.printf("  %-25s: %.3f\n", "pid.yaw.ki", _settings.pidYaw.ki / (float)PID_SCALE_FACTOR);
    Serial.printf("  %-25s: %.3f\n", "pid.yaw.kd", _settings.pidYaw.kd / (float)PID_SCALE_FACTOR);
    Serial.println();
    Serial.printf("  %-25s: %.2f\n", "pid.integral_limit", _settings.pidIntegralLimit);
    Serial.println("\n--- Rate Settings ---");
    Serial.printf("  %-25s: %.2f (deg)\n", "rates.angle", _settings.rates.maxAngleRollPitch);
    Serial.printf("  %-25s: %.2f (deg/s)\n", "rates.yaw", _settings.rates.maxRateYaw);
    Serial.printf("  %-25s: %.2f (deg/s)\n", "rates.acro", _settings.rates.maxRateRollPitch);
    Serial.println("\n--- Filter Settings ---");
    Serial.printf("  %-25s: %.1f (Hz)\n", "madgwick.sample_freq", _settings.filter.madgwickSampleFreq);
    Serial.printf("  %-25s: %.4f\n", "madgwick.beta", _settings.filter.madgwickBeta);
    Serial.println("\n--- Receiver Settings ---");
    Serial.printf("  %-25s: %d\n", "rx.min", _settings.receiver.ibusMinValue);
    Serial.printf("  %-25s: %d\n", "rx.max", _settings.receiver.ibusMaxValue);
    Serial.printf("  %-25s: %d\n", "rx.arming_threshold", _settings.receiver.armingThreshold);
    Serial.printf("  %-25s: %d\n", "rx.failsafe_threshold", _settings.receiver.failsafeThreshold);
    Serial.printf("  %-25s: %s\n", "rx.protocol", getReceiverProtocolString(_settings.receiverProtocol).c_str());
    Serial.println("\n--- IMU Settings ---");
    Serial.printf("  %-25s: %s\n", "imu.protocol", getImuProtocolString(_settings.imuProtocol).c_str());
    Serial.println("\n--- Receiver Channel Mapping ---");
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
        Serial.printf("  %-25s: %d\n", getFlightControlInputString((FlightControlInput)i).c_str(), _settings.channelMapping.channel[i]);
    }
    Serial.println("\n--- Motor Settings ---");
    Serial.printf("  %-25s: %.1f (%%)\n", "motor.idle_speed", _settings.motorIdleSpeedPercent);
    Serial.printf("  %-25s: %s\n", "motor.dshot_mode", getDShotModeString(_settings.dshotMode).c_str());
    Serial.println("\n--------------------------------------");
}

void CliCommandProcessor::handleDumpJsonCommand() {
    Serial.print("{\"settings\":{");
    Serial.print("\"pid.roll.kp\":"); Serial.print(_settings.pidRoll.kp / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.roll.ki\":"); Serial.print(_settings.pidRoll.ki / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.roll.kd\":"); Serial.print(_settings.pidRoll.kd / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.pitch.kp\":"); Serial.print(_settings.pidPitch.kp / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.pitch.ki\":"); Serial.print(_settings.pidPitch.ki / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.pitch.kd\":"); Serial.print(_settings.pidPitch.kd / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.yaw.kp\":"); Serial.print(_settings.pidYaw.kp / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.yaw.ki\":"); Serial.print(_settings.pidYaw.ki / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.yaw.kd\":"); Serial.print(_settings.pidYaw.kd / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"pid.integral_limit\":"); Serial.print(_settings.pidIntegralLimit, 4);
    Serial.print(",\"rates.angle\":"); Serial.print(_settings.rates.maxAngleRollPitch, 4);
    Serial.print(",\"rates.yaw\":"); Serial.print(_settings.rates.maxRateYaw, 4);
    Serial.print(",\"rates.acro\":"); Serial.print(_settings.rates.maxRateRollPitch, 4);
    Serial.print(",\"madgwick.sample_freq\":"); Serial.print(_settings.filter.madgwickSampleFreq, 1);
    Serial.print(",\"madgwick.beta\":"); Serial.print(_settings.filter.madgwickBeta, 4);
    Serial.print(",\"rx.min\":"); Serial.print(_settings.receiver.ibusMinValue);
    Serial.print(",\"rx.max\":"); Serial.print(_settings.receiver.ibusMaxValue);
    Serial.print(",\"rx.arming_threshold\":"); Serial.print(_settings.receiver.armingThreshold);
    Serial.print(",\"rx.failsafe_threshold\":"); Serial.print(_settings.receiver.failsafeThreshold);
    Serial.print(",\"rx.protocol\":"); Serial.print((int)_settings.receiverProtocol);
    Serial.print(",\"imu.protocol\":"); Serial.print((int)_settings.imuProtocol);
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
        String key = "rx.map.";
        String inputName = getFlightControlInputString((FlightControlInput)i);
        inputName.toLowerCase();
        key += inputName;
        Serial.print(",\""); Serial.print(key); Serial.print("\":"); Serial.print(_settings.channelMapping.channel[i]);
    }
    Serial.print(",\"motor.idle_speed\":"); Serial.print(_settings.motorIdleSpeedPercent, 1);
    Serial.print(",\"motor.dshot_mode\":\""); Serial.print(getDShotModeString(_settings.dshotMode)); Serial.print("\"");
    Serial.println("}}");
}

void CliCommandProcessor::printCliHelp() {
    Serial.println("\nAvailable commands:");
    Serial.printf("  %-22s - %s\n", "get <parameter>", "Get a specific setting value or category (e.g., 'get pid', 'get motor', 'get rx.channels').");
    Serial.printf("  %-22s - %s\n", "set <param> <value>", "Set a new value for a setting.");
    Serial.printf("  %-22s - %s\n", "dump", "Print all current settings.");
    Serial.printf("  %-22s - %s\n", "save", "Save current settings to flash memory.");
    Serial.printf("  %-22s - %s\n", "reset", "Reset all settings to defaults.");
    Serial.printf("  %-22s - %s\n", "reboot", "Reboot the flight controller.");
    Serial.printf("  %-22s - %s\n", "calibrate_imu", "Manually trigger IMU calibration.");
    Serial.printf("  %-22s - %s\n", "help", "Show this help message.");
    Serial.printf("  %-22s - %s\n", "exit", "Deactivate the CLI and return to flight mode.");
}

void CliCommandProcessor::triggerImuCalibration() {
    _fc->requestImuCalibration();
}

// --- Private Helper Functions for Printing Settings ---

void CliCommandProcessor::_printPidSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"pid\":{");
        Serial.print("\"roll\":{\"kp\":"); Serial.print(_settings.pidRoll.kp / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"ki\":"); Serial.print(_settings.pidRoll.ki / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"kd\":"); Serial.print(_settings.pidRoll.kd / (float)PID_SCALE_FACTOR, 3);
        Serial.print("},\n");
        Serial.print("\"pitch\":{\"kp\":"); Serial.print(_settings.pidPitch.kp / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"ki\":"); Serial.print(_settings.pidPitch.ki / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"kd\":"); Serial.print(_settings.pidPitch.kd / (float)PID_SCALE_FACTOR, 3);
        Serial.print("},\n");
        Serial.print("\"yaw\":{\"kp\":"); Serial.print(_settings.pidYaw.kp / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"ki\":"); Serial.print(_settings.pidYaw.ki / (float)PID_SCALE_FACTOR, 3);
        Serial.print(",\"kd\":"); Serial.print(_settings.pidYaw.kd / (float)PID_SCALE_FACTOR, 3);
        Serial.print("},\n");
        Serial.print("\"integral_limit\":"); Serial.print(_settings.pidIntegralLimit, 4);
        Serial.println("}}");
    } else {
        Serial.println("--- PID Settings ---");
        Serial.printf("  %-25s: %.3f\n", "pid.roll.kp", _settings.pidRoll.kp / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.roll.ki", _settings.pidRoll.ki / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.roll.kd", _settings.pidRoll.kd / (float)PID_SCALE_FACTOR);
        Serial.println();
        Serial.printf("  %-25s: %.3f\n", "pid.pitch.kp", _settings.pidPitch.kp / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.pitch.ki", _settings.pidPitch.ki / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.pitch.kd", _settings.pidPitch.kd / (float)PID_SCALE_FACTOR);
        Serial.println();
        Serial.printf("  %-25s: %.3f\n", "pid.yaw.kp", _settings.pidYaw.kp / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.yaw.ki", _settings.pidYaw.ki / (float)PID_SCALE_FACTOR);
        Serial.printf("  %-25s: %.3f\n", "pid.yaw.kd", _settings.pidYaw.kd / (float)PID_SCALE_FACTOR);
        Serial.println();
        Serial.printf("  %-25s: %.2f\n", "pid.integral_limit", _settings.pidIntegralLimit);
    }
}

void CliCommandProcessor::_printRatesSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"rates\":{");
        Serial.print("\"angle\":"); Serial.print(_settings.rates.maxAngleRollPitch, 4);
        Serial.print(",\"yaw\":"); Serial.print(_settings.rates.maxRateYaw, 4);
        Serial.print(",\"acro\":"); Serial.print(_settings.rates.maxRateRollPitch, 4);
        Serial.println("}}");
    } else {
        Serial.println("\n--- Rate Settings ---");
        Serial.printf("  %-25s: %.2f (deg)\n", "rates.angle", _settings.rates.maxAngleRollPitch);
        Serial.printf("  %-25s: %.2f (deg/s)\n", "rates.yaw", _settings.rates.maxRateYaw);
        Serial.printf("  %-25s: %.2f (deg/s)\n", "rates.acro", _settings.rates.maxRateRollPitch);
    }
}

void CliCommandProcessor::_printFilterSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"filter\":{");
        Serial.print("\"madgwick.sample_freq\":"); Serial.print(_settings.filter.madgwickSampleFreq, 1);
        Serial.print(",\"madgwick.beta\":"); Serial.print(_settings.filter.madgwickBeta, 4);
        Serial.println("}}");
    } else {
        Serial.println("\n--- Filter Settings ---");
        Serial.printf("  %-25s: %.1f (Hz)\n", "madgwick.sample_freq", _settings.filter.madgwickSampleFreq);
        Serial.printf("  %-25s: %.4f\n", "madgwick.beta", _settings.filter.madgwickBeta);
    }
}

void CliCommandProcessor::_printReceiverSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"receiver\":{");
        Serial.print("\"min\":"); Serial.print(_settings.receiver.ibusMinValue);
        Serial.print(",\"max\":"); Serial.print(_settings.receiver.ibusMaxValue);
        Serial.print(",\"arming_threshold\":"); Serial.print(_settings.receiver.armingThreshold);
        Serial.print(",\"failsafe_threshold\":"); Serial.print(_settings.receiver.failsafeThreshold);
        Serial.print(",\"protocol\":"); Serial.print((int)_settings.receiverProtocol);
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
            String key = "map.";
            String inputName = getFlightControlInputString((FlightControlInput)i);
            inputName.toLowerCase();
            key += inputName;
            Serial.print(",\""); Serial.print(key); Serial.print("\":"); Serial.print(_settings.channelMapping.channel[i]);
        }
        Serial.println("}}");
    } else {
        Serial.println("\n--- Receiver Settings ---");
        Serial.printf("  %-25s: %d\n", "rx.min", _settings.receiver.ibusMinValue);
        Serial.printf("  %-25s: %d\n", "rx.max", _settings.receiver.ibusMaxValue);
        Serial.printf("  %-25s: %d\n", "rx.arming_threshold", _settings.receiver.armingThreshold);
        Serial.printf("  %-25s: %d\n", "rx.failsafe_threshold", _settings.receiver.failsafeThreshold);
        Serial.printf("  %-25s: %s\n", "rx.protocol", getReceiverProtocolString(_settings.receiverProtocol).c_str());
        Serial.println("\n--- Receiver Channel Mapping ---");
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
            Serial.printf("  %-25s: %d\n", getFlightControlInputString((FlightControlInput)i).c_str(), _settings.channelMapping.channel[i]);
        }
    }
}

void CliCommandProcessor::_printImuSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"imu\":{");
        Serial.print("\"protocol\":"); Serial.print((int)_settings.imuProtocol);
        Serial.println("}}");
    } else {
        Serial.println("\n--- IMU Settings ---");
        Serial.printf("  %-25s: %s\n", "imu.protocol", getImuProtocolString(_settings.imuProtocol).c_str());
    }
}

void CliCommandProcessor::_printMotorSettings(bool isApiMode) {
    if (isApiMode) {
        Serial.print("{\"motor\":{");
        Serial.print("\"idle_speed\":"); Serial.print(_settings.motorIdleSpeedPercent, 1);
        Serial.print(",\"dshot_mode\":\""); Serial.print(getDShotModeString(_settings.dshotMode)); Serial.print("\"");
        Serial.println("}}");
    } else {
        Serial.println("\n--- Motor Settings ---");
        Serial.printf("  %-25s: %.1f (%%)\n", "motor.idle_speed", _settings.motorIdleSpeedPercent);
        Serial.printf("  %-25s: %s\n", "motor.dshot_mode", getDShotModeString(_settings.dshotMode).c_str());
    }
}
