#include <Arduino.h>
#include "cli.h"
#include "config.h"
#include "serial_logger.h"
#include "settings.h"

#include <Arduino.h>
#include "cli.h"
#include "config.h"
#include "serial_logger.h"
#include "settings.h"

static bool cliActive = false;
static bool apiActive = false;

// Forward declarations for helper functions
void printCliHelp();
CliCommand executeCliCommand(String command, const FlightState &state, bool isApiMode);
void handleGetCommand(String args, bool isApiMode);
void handleSetCommand(String args, bool isApiMode);
void handleDumpCommand();
void handleDumpJsonCommand();

CliCommand handleSerialCli(const FlightState &state)
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0) return CliCommand::NONE;

        // Mode selection
        if (!cliActive && !apiActive) {
            if (input.equalsIgnoreCase("cli")) {
                cliActive = true;
                apiActive = false;
                Serial.println("--- CLI Activated ---");
                Serial.print("ESP32_FC > ");
            } else if (input.equalsIgnoreCase("api")) {
                apiActive = true;
                cliActive = false;
                Serial.println("{\"status\":\"api_mode_activated\"}");
            }
            return CliCommand::NONE;
        }

        // Command execution within a mode
        if (cliActive)
        {
            String commandName = input;
            int firstSpace = input.indexOf(' ');
            if (firstSpace != -1) {
                commandName = input.substring(0, firstSpace);
            }

            if (commandName.equalsIgnoreCase("exit")) {
                cliActive = false;
                Serial.println("--- CLI Deactivated ---");
                return CliCommand::NONE;
            }

            CliCommand cmd = executeCliCommand(input, state, false);

            if (!commandName.equalsIgnoreCase("save") && !commandName.equalsIgnoreCase("reboot") && !commandName.equalsIgnoreCase("reset")) {
                Serial.print("ESP32_FC > ");
            }
            return cmd;
        }
        else if (apiActive)
        {
            // In API mode, just execute the command. No prompts.
            // The web client is responsible for disconnecting to exit API mode.
            return executeCliCommand(input, state, true);
        }
    }
    return CliCommand::NONE;
}

void printCliHelp()
{
    Serial.println("\nAvailable commands:");
    Serial.printf("  %-22s - %s\n", "get <parameter>", "Get a specific setting value.");
    Serial.printf("  %-22s - %s\n", "set <param> <value>", "Set a new value for a setting.");
    Serial.printf("  %-22s - %s\n", "dump", "Print all current settings.");
    Serial.printf("  %-22s - %s\n", "dumpjson", "Print all current settings in JSON format.");
    Serial.printf("  %-22s - %s\n", "debug on/off", "Start/stop continuous flight data output (500ms interval).");
    Serial.printf("  %-22s - %s\n", "save", "Save current settings to flash memory.");
    Serial.printf("  %-22s - %s\n", "reset", "Reset all settings to defaults.");
    Serial.printf("  %-22s - %s\n", "reboot", "Reboot the flight controller.");
    Serial.printf("  %-22s - %s\n", "calibrate_imu", "Manually trigger IMU calibration.");
    Serial.printf("  %-22s - %s\n", "help", "Show this help message.");
    Serial.printf("  %-22s - %s\n", "exit", "Deactivate the CLI.");
}

CliCommand executeCliCommand(String command, const FlightState &state, bool isApiMode)
{
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

    if (commandName.equals("get")) {
        handleGetCommand(commandArgs, isApiMode);
    } else if (commandName.equals("set")) {
        handleSetCommand(commandArgs, isApiMode);
    } else if (commandName.equals("dump")) {
        handleDumpCommand();
    } else if (commandName.equals("dumpjson")) {
        handleDumpJsonCommand();
    } else if (commandName.equals("debug")) {
        if (commandArgs.equals("on")) {
            settings.enableLogging = true;
            settings.printIntervalMs = 500;
            if (!isApiMode) Serial.println("INFO: Continuous debug output enabled (500ms interval).");
        } else if (commandArgs.equals("off")) {
            settings.enableLogging = false;
            settings.printIntervalMs = 0;
            if (!isApiMode) Serial.println("INFO: Continuous debug output disabled.");
        } else {
            if (!isApiMode) Serial.println("ERROR: Invalid 'debug' command. Use 'debug on' or 'debug off'.");
        }
    } else if (commandName.equals("save")) {
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
        return CliCommand::CALIBRATE_IMU;
    } else if (commandName.equals("help")) {
        if (!isApiMode) printCliHelp();
    } else {
        if (!isApiMode) {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        } else {
            Serial.println("{\"error\":\"Unknown command\"}");
        }
    }
    return CliCommand::NONE;
}

void handleGetCommand(String args, bool isApiMode) {
    // In API mode, get commands should return JSON
    // For now, we keep the simple text output as the webapp doesn't use 'get' yet.
    if (args.equals("pid.roll.kp")) Serial.println(settings.pidRoll.kp / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.roll.ki")) Serial.println(settings.pidRoll.ki / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.roll.kd")) Serial.println(settings.pidRoll.kd / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.pitch.kp")) Serial.println(settings.pidPitch.kp / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.pitch.ki")) Serial.println(settings.pidPitch.ki / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.pitch.kd")) Serial.println(settings.pidPitch.kd / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.yaw.kp")) Serial.println(settings.pidYaw.kp / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.yaw.ki")) Serial.println(settings.pidYaw.ki / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.yaw.kd")) Serial.println(settings.pidYaw.kd / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.integral_limit")) Serial.println(settings.pidIntegralLimit, 4);
    else if (args.equals("rates.angle")) Serial.println(settings.rates.maxAngleRollPitch, 4);
    else if (args.equals("rates.yaw")) Serial.println(settings.rates.maxRateYaw, 4);
    else if (args.equals("rates.acro")) Serial.println(settings.rates.maxRateRollPitch, 4);
    else if (args.equals("madgwick.sample_freq")) Serial.println(settings.filter.madgwickSampleFreq, 1);
    else if (args.equals("madgwick.beta")) Serial.println(settings.filter.madgwickBeta, 4);
    else if (args.equals("rx.min")) Serial.println(settings.receiver.ibusMinValue);
    else if (args.equals("rx.max")) Serial.println(settings.receiver.ibusMaxValue);
    else if (args.equals("rx.arming_threshold")) Serial.println(settings.receiver.armingThreshold);
    else if (args.equals("rx.failsafe_threshold")) Serial.println(settings.receiver.failsafeThreshold);
    else if (args.equals("rx.protocol")) Serial.println((int)settings.receiverProtocol);
    else if (args.equals("imu.protocol")) Serial.println((int)settings.imuProtocol);
    else if (args.startsWith("rx.map.")) {
        String inputName = args.substring(7);
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
            if (inputName.equalsIgnoreCase(getFlightControlInputString((FlightControlInput)i))) {
                Serial.println(settings.channelMapping.channel[i]);
                return;
            }
        }
        if (!isApiMode) Serial.println("Unknown flight control input for rx.map.");
    } else if (args.equals("motor.idle_speed")) {
        Serial.println(settings.motorIdleSpeedPercent, 1);
    } else {
        if (!isApiMode) Serial.println("Unknown or unsupported parameter for 'get'.");
    }
}

void handleSetCommand(String args, bool isApiMode) {
    int lastSpace = args.lastIndexOf(' ');
    if (lastSpace == -1) {
        if (!isApiMode) Serial.println("Invalid 'set' format. Use: set <parameter> <value>");
        return;
    }

    String param = args.substring(0, lastSpace);
    String valueStr = args.substring(lastSpace + 1);

    // PID (int, scaled)
    if (param.equals("pid.roll.kp")) settings.pidRoll.kp = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.roll.ki")) settings.pidRoll.ki = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.roll.kd")) settings.pidRoll.kd = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.kp")) settings.pidPitch.kp = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.ki")) settings.pidPitch.ki = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.kd")) settings.pidPitch.kd = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.kp")) settings.pidYaw.kp = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.ki")) settings.pidYaw.ki = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.kd")) settings.pidYaw.kd = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.integral_limit")) settings.pidIntegralLimit = valueStr.toFloat();
    // Rates (float)
    else if (param.equals("rates.angle")) settings.rates.maxAngleRollPitch = valueStr.toFloat();
    else if (param.equals("rates.yaw")) settings.rates.maxRateYaw = valueStr.toFloat();
    else if (param.equals("rates.acro")) settings.rates.maxRateRollPitch = valueStr.toFloat();
    // Filter (float)
    else if (param.equals("madgwick.sample_freq")) settings.filter.madgwickSampleFreq = valueStr.toFloat();
    else if (param.equals("madgwick.beta")) settings.filter.madgwickBeta = valueStr.toFloat();
    // Receiver (int)
    else if (param.equals("rx.min")) settings.receiver.ibusMinValue = valueStr.toInt();
    else if (param.equals("rx.max")) settings.receiver.ibusMaxValue = valueStr.toInt();
    else if (param.equals("rx.arming_threshold")) settings.receiver.armingThreshold = valueStr.toInt();
    else if (param.equals("rx.failsafe_threshold")) settings.receiver.failsafeThreshold = valueStr.toInt();
    else if (param.equals("rx.protocol")) {
        int protocol = valueStr.toInt();
        if (protocol >= 0 && protocol < RECEIVER_PROTOCOL_COUNT) settings.receiverProtocol = (ReceiverProtocol)protocol;
        else if (!isApiMode) Serial.printf("Invalid receiver protocol. Use 0 for %s, 1 for %s.\n", getReceiverProtocolString(PROTOCOL_IBUS).c_str(), getReceiverProtocolString(PROTOCOL_PPM).c_str());
    } else if (param.equals("imu.protocol")) {
        int protocol = valueStr.toInt();
        if (protocol >= 0 && protocol < IMU_PROTOCOL_COUNT) settings.imuProtocol = (ImuProtocol)protocol;
        else if (!isApiMode) Serial.printf("Invalid IMU protocol. Use 0 for %s.\n", getImuProtocolString(IMU_MPU6050).c_str());
    } else if (param.startsWith("rx.map.")) {
        String inputName = param.substring(7);
        int channelValue = valueStr.toInt();
        bool mapping_found = false;
        if (channelValue >= 0 && channelValue < RECEIVER_CHANNEL_COUNT) {
            for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i) {
                if (inputName.equalsIgnoreCase(getFlightControlInputString((FlightControlInput)i))) {
                    settings.channelMapping.channel[i] = channelValue;
                    if (!isApiMode) Serial.printf("Mapped %s to channel %d.\n", getFlightControlInputString((FlightControlInput)i).c_str(), channelValue);
                    mapping_found = true;
                    break;
                }
            }
            if (!mapping_found && !isApiMode) Serial.println("Unknown flight control input for rx.map.");
        } else {
            if (!isApiMode) Serial.printf("Invalid channel number. Must be between 0 and %d.\n", RECEIVER_CHANNEL_COUNT - 1);
        }
    } else if (param.equals("motor.idle_speed")) {
        settings.motorIdleSpeedPercent = round(valueStr.toFloat() * 10.0f) / 10.0f;
    } else {
        if (!isApiMode) Serial.println("Unknown or unsupported parameter for 'set'.");
        return;
    }

    if (!isApiMode) {
        Serial.print("Set ");
        Serial.print(param);
        Serial.print(" to ");
        Serial.println(valueStr);
    }
}

// Helper function to convert ReceiverProtocol enum to string
String getReceiverProtocolString(ReceiverProtocol protocol)
{
    switch (protocol)
    {
    case PROTOCOL_IBUS:
        return "IBUS";
    case PROTOCOL_PPM:
        return "PPM";
    default:
        return "UNKNOWN";
    }
}

// Helper function to convert ImuProtocol enum to string
String getImuProtocolString(ImuProtocol protocol)
{
    switch (protocol)
    {
    case IMU_MPU6050:
        return "MPU6050";
    default:
        return "UNKNOWN";
    }
}

// Helper function to convert FlightControlInput enum to string
String getFlightControlInputString(FlightControlInput input)
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

void handleDumpCommand()
{
    Serial.println("\n--- [ Flight Controller Settings ] ---\n");

    // PID Settings
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

    // Rate Settings
    Serial.println("\n--- Rate Settings ---");
    Serial.printf("  %-25s: %.2f (deg)\n", "rates.angle", settings.rates.maxAngleRollPitch);
    Serial.printf("  %-25s: %.2f (deg/s)\n", "rates.yaw", settings.rates.maxRateYaw);
    Serial.printf("  %-25s: %.2f (deg/s)\n", "rates.acro", settings.rates.maxRateRollPitch);

    // Filter Settings
    Serial.println("\n--- Filter Settings ---");
    Serial.printf("  %-25s: %.1f (Hz)\n", "madgwick.sample_freq", settings.filter.madgwickSampleFreq);
    Serial.printf("  %-25s: %.4f\n", "madgwick.beta", settings.filter.madgwickBeta);

    // Receiver Settings
    Serial.println("\n--- Receiver Settings ---");
    Serial.printf("  %-25s: %d\n", "rx.min", settings.receiver.ibusMinValue);
    Serial.printf("  %-25s: %d\n", "rx.max", settings.receiver.ibusMaxValue);
    Serial.printf("  %-25s: %d\n", "rx.arming_threshold", settings.receiver.armingThreshold);
    Serial.printf("  %-25s: %d\n", "rx.failsafe_threshold", settings.receiver.failsafeThreshold);
    Serial.printf("  %-25s: %s\n", "rx.protocol", getReceiverProtocolString(settings.receiverProtocol).c_str());

    // IMU Settings
    Serial.println("\n--- IMU Settings ---");
    Serial.printf("  %-25s: %s\n", "imu.protocol", getImuProtocolString(settings.imuProtocol).c_str());

    // Receiver Channel Mapping
    Serial.println("\n--- Receiver Channel Mapping ---");
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
    {
        Serial.printf("  %-25s: %d\n", getFlightControlInputString((FlightControlInput)i).c_str(), settings.channelMapping.channel[i]);
    }

    // Motor Settings
    Serial.println("\n--- Motor Settings ---");
    Serial.printf("  %-25s: %.1f (%%)\n", "motor.idle_speed", settings.motorIdleSpeedPercent);

    Serial.println("\n--------------------------------------");
}


void handleDumpJsonCommand()
{
    Serial.print("{\"settings\":{");

    // PID Settings
    Serial.print("\"pid\":{");
    Serial.print("\"roll\":{\"kp\":");
    Serial.print(settings.pidRoll.kp / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"ki\":");
    Serial.print(settings.pidRoll.ki / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"kd\":");
    Serial.print(settings.pidRoll.kd / (float)PID_SCALE_FACTOR, 3);
    Serial.print("},");

    Serial.print("\"pitch\":{\"kp\":");
    Serial.print(settings.pidPitch.kp / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"ki\":");
    Serial.print(settings.pidPitch.ki / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"kd\":");
    Serial.print(settings.pidPitch.kd / (float)PID_SCALE_FACTOR, 3);
    Serial.print("},");

    Serial.print("\"yaw\":{\"kp\":");
    Serial.print(settings.pidYaw.kp / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"ki\":");
    Serial.print(settings.pidYaw.ki / (float)PID_SCALE_FACTOR, 3);
    Serial.print(",\"kd\":");
    Serial.print(settings.pidYaw.kd / (float)PID_SCALE_FACTOR, 3);
    Serial.print("}");

    Serial.print(",\"integral_limit\":");
    Serial.print(settings.pidIntegralLimit, 2);
    Serial.print("},");

    // Rate Settings
    Serial.print("\"rates\":{");
    Serial.print("\"angle\":");
    Serial.print(settings.rates.maxAngleRollPitch, 2);
    Serial.print(",\"yaw\":");
    Serial.print(settings.rates.maxRateYaw, 2);
    Serial.print(",\"acro\":");
    Serial.print(settings.rates.maxRateRollPitch, 2);
    Serial.print("},");

    // Filter Settings
    Serial.print("\"filter\":{");
    Serial.print("\"madgwick_sample_freq\":");
    Serial.print(settings.filter.madgwickSampleFreq, 1);
    Serial.print(",\"madgwick_beta\":");
    Serial.print(settings.filter.madgwickBeta, 4);
    Serial.print("},");

    // Receiver Settings
    Serial.print("\"receiver\":{");
    Serial.print("\"min\":");
    Serial.print(settings.receiver.ibusMinValue);
    Serial.print(",\"max\":");
    Serial.print(settings.receiver.ibusMaxValue);
    Serial.print(",\"arming_threshold\":");
    Serial.print(settings.receiver.armingThreshold);
    Serial.print(",\"failsafe_threshold\":");
    Serial.print(settings.receiver.failsafeThreshold);
    Serial.print(",\"protocol\":");
    Serial.print((int)settings.receiverProtocol);
    Serial.print("},");

    // IMU Settings
    Serial.print("\"imu\":{");
    Serial.print("\"protocol\":");
    Serial.print((int)settings.imuProtocol);
    Serial.print("},");

    // Channel Mapping
    Serial.print("\"channel_mapping\":{");
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
    {
        Serial.print("\"");
        Serial.print(getFlightControlInputString((FlightControlInput)i));
        Serial.print("\":");
        Serial.print(settings.channelMapping.channel[i]);
        if (i < NUM_FLIGHT_CONTROL_INPUTS - 1) {
            Serial.print(",");
        }
    }
    Serial.print("},");

    // Motor Settings
    Serial.print("\"motor\":{");
    Serial.print("\"idle_speed\":");
    Serial.print(settings.motorIdleSpeedPercent, 1);
    Serial.print("}");

    Serial.println("}}");
}