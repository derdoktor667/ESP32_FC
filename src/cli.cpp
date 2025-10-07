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
        delay(100);
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
        Serial.println(settings.pidRoll.kp, 4);
    else if (args.equals("pid.roll.ki"))
        Serial.println(settings.pidRoll.ki, 4);
    else if (args.equals("pid.roll.kd"))
        Serial.println(settings.pidRoll.kd, 4);
    else if (args.equals("pid.pitch.kp"))
        Serial.println(settings.pidPitch.kp, 4);
    else if (args.equals("pid.pitch.ki"))
        Serial.println(settings.pidPitch.ki, 4);
    else if (args.equals("pid.pitch.kd"))
        Serial.println(settings.pidPitch.kd, 4);
    else if (args.equals("pid.yaw.kp"))
        Serial.println(settings.pidYaw.kp, 4);
    else if (args.equals("pid.yaw.ki"))
        Serial.println(settings.pidYaw.ki, 4);
    else if (args.equals("pid.yaw.kd"))
        Serial.println(settings.pidYaw.kd, 4);
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
    else if (args.equals("filter.gain"))
        Serial.println(settings.filter.complementaryFilterGain, 4);
    // Receiver
    else if (args.equals("rx.min"))
        Serial.println(settings.receiver.ibusMinValue);
    else if (args.equals("rx.max"))
        Serial.println(settings.receiver.ibusMaxValue);
    else if (args.equals("rx.arming_threshold"))
        Serial.println(settings.receiver.armingThreshold);
    else if (args.equals("rx.failsafe_threshold"))
        Serial.println(settings.receiver.failsafeThreshold);
    // Logging
    else if (args.equals("log.interval"))
        Serial.println(settings.printIntervalMs);
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

    // PID (float)
    if (param.equals("pid.roll.kp"))
        settings.pidRoll.kp = valueStr.toFloat();
    else if (param.equals("pid.roll.ki"))
        settings.pidRoll.ki = valueStr.toFloat();
    else if (param.equals("pid.roll.kd"))
        settings.pidRoll.kd = valueStr.toFloat();
    else if (param.equals("pid.pitch.kp"))
        settings.pidPitch.kp = valueStr.toFloat();
    else if (param.equals("pid.pitch.ki"))
        settings.pidPitch.ki = valueStr.toFloat();
    else if (param.equals("pid.pitch.kd"))
        settings.pidPitch.kd = valueStr.toFloat();
    else if (param.equals("pid.yaw.kp"))
        settings.pidYaw.kp = valueStr.toFloat();
    else if (param.equals("pid.yaw.ki"))
        settings.pidYaw.ki = valueStr.toFloat();
    else if (param.equals("pid.yaw.kd"))
        settings.pidYaw.kd = valueStr.toFloat();
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
    else if (param.equals("filter.gain"))
        settings.filter.complementaryFilterGain = valueStr.toFloat();
    // Receiver (int)
    else if (param.equals("rx.min"))
        settings.receiver.ibusMinValue = valueStr.toInt();
    else if (param.equals("rx.max"))
        settings.receiver.ibusMaxValue = valueStr.toInt();
    else if (param.equals("rx.arming_threshold"))
        settings.receiver.armingThreshold = valueStr.toInt();
    else if (param.equals("rx.failsafe_threshold"))
        settings.receiver.failsafeThreshold = valueStr.toInt();
    // Logging (unsigned long)
    else if (param.equals("log.interval"))
        settings.printIntervalMs = valueStr.toInt();
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

void handleDumpCommand()
{
    Serial.println("\n--- [ Flight Controller Settings ] ---\n");

    // PID Settings
    Serial.println("--- PID Settings ---");
    Serial.printf("  %-25s: %.4f\n", "pid.roll.kp", settings.pidRoll.kp);
    Serial.printf("  %-25s: %.4f\n", "pid.roll.ki", settings.pidRoll.ki);
    Serial.printf("  %-25s: %.4f\n", "pid.roll.kd", settings.pidRoll.kd);
    Serial.println();
    Serial.printf("  %-25s: %.4f\n", "pid.pitch.kp", settings.pidPitch.kp);
    Serial.printf("  %-25s: %.4f\n", "pid.pitch.ki", settings.pidPitch.ki);
    Serial.printf("  %-25s: %.4f\n", "pid.pitch.kd", settings.pidPitch.kd);
    Serial.println();
    Serial.printf("  %-25s: %.4f\n", "pid.yaw.kp", settings.pidYaw.kp);
    Serial.printf("  %-25s: %.4f\n", "pid.yaw.ki", settings.pidYaw.ki);
    Serial.printf("  %-25s: %.4f\n", "pid.yaw.kd", settings.pidYaw.kd);
    Serial.println();
    Serial.printf("  %-25s: %.2f\n", "pid.integral_limit", settings.pidIntegralLimit);

    // Rate Settings
    Serial.println("\n--- Rate Settings ---");
    Serial.printf("  %-25s: %.2f (deg)\n", "rates.angle", settings.rates.maxAngleRollPitch);
    Serial.printf("  %-25s: %.2f (deg/s)\n", "rates.yaw", settings.rates.maxRateYaw);
    Serial.printf("  %-25s: %.2f (deg/s)\n", "rates.acro", settings.rates.maxRateRollPitch);

    // Filter Settings
    Serial.println("\n--- Filter Settings ---");
    Serial.printf("  %-25s: %.2f\n", "filter.gain", settings.filter.complementaryFilterGain);

    // Receiver Settings
    Serial.println("\n--- Receiver Settings ---");
    Serial.printf("  %-25s: %d\n", "rx.min", settings.receiver.ibusMinValue);
    Serial.printf("  %-25s: %d\n", "rx.max", settings.receiver.ibusMaxValue);
    Serial.printf("  %-25s: %d\n", "rx.arming_threshold", settings.receiver.armingThreshold);
    Serial.printf("  %-25s: %d\n", "rx.failsafe_threshold", settings.receiver.failsafeThreshold);

    // Logging Settings
    Serial.println("\n--- Logging Settings ---");
    Serial.printf("  %-25s: %lu (ms)\n", "log.interval", settings.printIntervalMs);

    Serial.println("\n--------------------------------------");
}