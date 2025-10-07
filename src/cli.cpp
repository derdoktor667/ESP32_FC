#include <Arduino.h>
#include "cli.h"
#include "config.h"
#include "serial_logger.h"
#include "settings.h"

static bool cliActive = false;

// Forward declarations for helper functions
void printCliHeader();
void printCliHelp();
CliCommand executeCliCommand(String command, const FlightState &state); // Updated signature
void handleGetCommand(String args);
void handleSetCommand(String args);
void handleDumpCommand();

// Updated to accept the flight state and return a CliCommand
CliCommand handleSerialCli(const FlightState &state)
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (!cliActive && input.equalsIgnoreCase("cli"))
        {
            cliActive = true;
            printCliHeader();
            printCliHelp();
            return CliCommand::NONE; // CLI activated, no command to execute yet
        }

        if (cliActive)
        {
            if (input.equalsIgnoreCase("exit"))
            {
                cliActive = false;
                Serial.println("--- CLI Deactivated ---");
                return CliCommand::NONE;
            }
            return executeCliCommand(input, state); // Pass the state down and return its result
        }
    }
    return CliCommand::NONE; // No command or CLI not active
}

void printCliHeader()
{
    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("    ______ _____ ____    _______  __");
    Serial.println("   / ____// ___// __ \  / ____/ |/ /");
    Serial.println("  / __/   \__ \/ / / / / /_   |   / ");
    Serial.println(" / /___  ___/ / /_/ / / __/  /   |  ");
    Serial.println("/_____/ /____/\____/ /_/    /_/|_|  ");
    Serial.println("         Flight Controller CLI         ");
    Serial.println("========================================");
}

void printCliHelp()
{
    Serial.println("\nAvailable commands:");
    Serial.printf("  %-22s - %s\n", "get <parameter>", "Get a specific setting value.");
    Serial.printf("  %-22s - %s\n", "set <param> <value>", "Set a new value for a setting.");
    Serial.printf("  %-22s - %s\n", "dump", "Print all current settings.");
    Serial.printf("  %-22s - %s\n", "debug", "Print a snapshot of runtime flight data.");
    Serial.printf("  %-22s - %s\n", "save", "Save current settings to flash memory.");
    Serial.printf("  %-22s - %s\n", "reset", "Reset all settings to defaults.");
    Serial.printf("  %-22s - %s\n", "reboot", "Reboot the flight controller.");
    Serial.printf("  %-22s - %s\n", "calibrate_imu", "Manually trigger IMU calibration.");
    Serial.printf("  %-22s - %s\n", "motor.idle_speed", "Get/Set motor idle speed percentage (e.g., 4.0).");
    Serial.printf("  %-22s - %s (%d:%s, %d:%s)\n", "rx.protocol", "Get/Set receiver protocol", (int)PROTOCOL_IBUS, getReceiverProtocolString(PROTOCOL_IBUS).c_str(), (int)PROTOCOL_PPM, getReceiverProtocolString(PROTOCOL_PPM).c_str());
    Serial.printf("  %-22s - %s (%d:%s)\n", "imu.protocol", "Get/Set IMU protocol", (int)IMU_MPU6050, getImuProtocolString(IMU_MPU6050).c_str());
    Serial.printf("  %-22s - %s\n", "madgwick.sample_freq", "Get/Set Madgwick filter sample frequency (Hz).");
    Serial.printf("  %-22s - %s\n", "madgwick.beta", "Get/Set Madgwick filter beta parameter.");
    Serial.printf("  %-22s - %s\n", "rx.map.<input>", "Get/Set receiver channel for a flight control input (e.g., rx.map.roll 0).");
    Serial.printf("  %-22s - %s\n", "help", "Show this help message.");
    Serial.printf("  %-22s - %s\n", "exit", "Deactivate the CLI.");
    Serial.print("\nESP32_FC > ");
}

// Updated to accept the flight state and return a CliCommand
CliCommand executeCliCommand(String command, const FlightState &state)
{
    command.toLowerCase();
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

    if (commandName.equals("get"))
    {
        handleGetCommand(commandArgs);
    }
    else if (commandName.equals("set"))
    {
        handleSetCommand(commandArgs);
    }
    else if (commandName.equals("dump"))
    {
        handleDumpCommand();
    }
    else if (commandName.equals("debug"))
    {
        printFlightStatus(state); // Pass the state to the logger
    }
    else if (commandName.equals("save"))
    {
        saveSettings();
        Serial.println("INFO: Settings saved. Rebooting...");
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    }
    else if (commandName.equals("reset"))
    {
        settings = FlightControllerSettings();
        saveSettings();
        Serial.println("INFO: All settings have been reset to their default values and saved.");
    }
    else if (commandName.equals("reboot"))
    {
        Serial.println("Rebooting...");
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    }
    else if (commandName.equals("calibrate_imu"))
    {
        Serial.println("INFO: IMU calibration requested.");
        return CliCommand::CALIBRATE_IMU; // Request calibration from FlightController
    }
    else if (commandName.equals("help"))
    {
        printCliHelp();
    }
    else
    {
        Serial.print("Unknown command: ");
        Serial.println(commandName);
    }
    return CliCommand::NONE; // No special command requested
}

void handleGetCommand(String args)
{
    // PID
    if (args.equals("pid.roll.kp"))
        Serial.println(settings.pidRoll.kp / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.roll.ki"))
        Serial.println(settings.pidRoll.ki / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.roll.kd"))
        Serial.println(settings.pidRoll.kd / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.pitch.kp"))
        Serial.println(settings.pidPitch.kp / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.pitch.ki"))
        Serial.println(settings.pidPitch.ki / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.pitch.kd"))
        Serial.println(settings.pidPitch.kd / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.yaw.kp"))
        Serial.println(settings.pidYaw.kp / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.yaw.ki"))
        Serial.println(settings.pidYaw.ki / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.yaw.kd"))
        Serial.println(settings.pidYaw.kd / (float)PID_SCALE_FACTOR, 3);
    else if (args.equals("pid.integral_limit"))
        Serial.println(settings.pidIntegralLimit, 4);
    // Rates
    else if (args.equals("rates.angle"))
        Serial.println(settings.rates.maxAngleRollPitch, 4);
    else if (args.equals("rates.yaw"))
        Serial.println(settings.rates.maxRateYaw, 4);
    else if (args.equals("rates.acro"))
        Serial.println(settings.rates.maxRateRollPitch, 4);
    // Filter
    else if (args.equals("madgwick.sample_freq"))
        Serial.println(settings.filter.madgwickSampleFreq, 1);
    else if (args.equals("madgwick.beta"))
        Serial.println(settings.filter.madgwickBeta, 4);
    // Receiver
    else if (args.equals("rx.min"))
        Serial.println(settings.receiver.ibusMinValue);
    else if (args.equals("rx.max"))
        Serial.println(settings.receiver.ibusMaxValue);
    else if (args.equals("rx.arming_threshold"))
        Serial.println(settings.receiver.armingThreshold);
    else if (args.equals("rx.failsafe_threshold"))
        Serial.println(settings.receiver.failsafeThreshold);
    else if (args.equals("rx.protocol"))
        Serial.println((int)settings.receiverProtocol);
    // IMU
    else if (args.equals("imu.protocol"))
        Serial.println((int)settings.imuProtocol);
    // Channel Mapping
    else if (args.startsWith("rx.map."))
    {
        String inputName = args.substring(7); // Remove "rx.map."
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
        {
            if (inputName.equalsIgnoreCase(getFlightControlInputString((FlightControlInput)i)))
            {
                Serial.println(settings.channelMapping.channel[i]);
                return;
            }
        }
        Serial.println("Unknown flight control input for rx.map.");
    }
    // Logging
    else if (args.equals("log.interval"))
        Serial.println(settings.printIntervalMs);
    // Motor
    else if (args.equals("motor.idle_speed"))
        Serial.println(settings.motorIdleSpeedPercent, 1);
    else
    {
        Serial.println("Unknown or unsupported parameter for 'get'.");
    }
}

void handleSetCommand(String args)
{
    int lastSpace = args.lastIndexOf(' ');
    if (lastSpace == -1)
    {
        Serial.println("Invalid 'set' format. Use: set <parameter> <value>");
        return;
    }

    String param = args.substring(0, lastSpace);
    String valueStr = args.substring(lastSpace + 1);

    // PID (int, scaled)
    if (param.equals("pid.roll.kp"))
        settings.pidRoll.kp = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.roll.ki"))
        settings.pidRoll.ki = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.roll.kd"))
        settings.pidRoll.kd = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.kp"))
        settings.pidPitch.kp = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.ki"))
        settings.pidPitch.ki = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.pitch.kd"))
        settings.pidPitch.kd = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.kp"))
        settings.pidYaw.kp = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.ki"))
        settings.pidYaw.ki = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.yaw.kd"))
        settings.pidYaw.kd = round(valueStr.toFloat() * PID_SCALE_FACTOR);
    else if (param.equals("pid.integral_limit"))
        settings.pidIntegralLimit = valueStr.toFloat();
    // Rates (float)
    else if (param.equals("rates.angle"))
        settings.rates.maxAngleRollPitch = valueStr.toFloat();
    else if (param.equals("rates.yaw"))
        settings.rates.maxRateYaw = valueStr.toFloat();
    else if (param.equals("rates.acro"))
        settings.rates.maxRateRollPitch = valueStr.toFloat();
    // Filter (float)
    else if (param.equals("madgwick.sample_freq"))
        settings.filter.madgwickSampleFreq = valueStr.toFloat();
    else if (param.equals("madgwick.beta"))
        settings.filter.madgwickBeta = valueStr.toFloat();
    // Receiver (int)
    else if (param.equals("rx.min"))
        settings.receiver.ibusMinValue = valueStr.toInt();
    else if (param.equals("rx.max"))
        settings.receiver.ibusMaxValue = valueStr.toInt();
    else if (param.equals("rx.arming_threshold"))
        settings.receiver.armingThreshold = valueStr.toInt();
    else if (param.equals("rx.failsafe_threshold"))
        settings.receiver.failsafeThreshold = valueStr.toInt();
    else if (param.equals("rx.protocol"))
    {
        int protocol = valueStr.toInt();
        if (protocol >= 0 && protocol < RECEIVER_PROTOCOL_COUNT)
            settings.receiverProtocol = (ReceiverProtocol)protocol;
        else
            Serial.printf("Invalid receiver protocol. Use 0 for %s, 1 for %s.\n", getReceiverProtocolString(PROTOCOL_IBUS).c_str(), getReceiverProtocolString(PROTOCOL_PPM).c_str());
    }
    // IMU (int)
    else if (param.equals("imu.protocol"))
    {
        int protocol = valueStr.toInt();
        if (protocol >= 0 && protocol < IMU_PROTOCOL_COUNT)
            settings.imuProtocol = (ImuProtocol)protocol;
        else
            Serial.printf("Invalid IMU protocol. Use 0 for %s.\n", getImuProtocolString(IMU_MPU6050).c_str());
    }
    // Channel Mapping (int)
    else if (param.startsWith("rx.map."))
    {
        String inputName = param.substring(7); // Remove "rx.map."
        int channelValue = valueStr.toInt();
        if (channelValue >= 0 && channelValue < RECEIVER_CHANNEL_COUNT)
        {
            for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
            {
                if (inputName.equalsIgnoreCase(getFlightControlInputString((FlightControlInput)i)))
                {
                    settings.channelMapping.channel[i] = channelValue;
                    Serial.printf("Mapped %s to channel %d.\n", getFlightControlInputString((FlightControlInput)i).c_str(), channelValue);
                    return; // Exit after successful mapping
                }
            }
            Serial.println("Unknown flight control input for rx.map.");
        }
        else
        {
            Serial.printf("Invalid channel number. Must be between 0 and %d.\n", RECEIVER_CHANNEL_COUNT - 1);
        }
    }
    // Logging (unsigned long)
    else if (param.equals("log.interval"))
        settings.printIntervalMs = valueStr.toInt();
    // Motor (float)
    else if (param.equals("motor.idle_speed"))
        settings.motorIdleSpeedPercent = round(valueStr.toFloat() * 10.0f) / 10.0f;
    else
    {
        Serial.println("Unknown or unsupported parameter for 'set'.");
        return; // Don't print the confirmation message
    }

    Serial.print("Set ");
    Serial.print(param);
    Serial.print(" to ");
    Serial.println(valueStr);
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

    // Logging Settings
    Serial.println("\n--- Logging Settings ---");
    Serial.printf("  %-25s: %lu (ms)\n", "log.interval", settings.printIntervalMs);

    // Motor Settings
    Serial.println("\n--- Motor Settings ---");
    Serial.printf("  %-25s: %.1f (%%)\n", "motor.idle_speed", settings.motorIdleSpeedPercent);

    Serial.println("\n--------------------------------------");
}