#include <Arduino.h>
#include "cli.h"
#include "config.h"
#include "serial_logger.h" // For the debug command
#include "settings.h"      // For save/load functionality

static bool cliActive = false;

// Forward declarations for helper functions
void printCliHelp();
void executeCliCommand(String command);
void handleGetCommand(String args);
void handleSetCommand(String args);

void handleSerialCli()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (!cliActive && input.equalsIgnoreCase("cli"))
        {
            cliActive = true;
            Serial.println("\n\n--- ESP32_FC CLI Activated ---");
            printCliHelp();
            return; // Wait for next input
        }

        if (cliActive)
        {
            if (input.equalsIgnoreCase("exit"))
            {
                cliActive = false;
                Serial.println("--- CLI Deactivated ---");
                return;
            }
            executeCliCommand(input);
            Serial.print("CLI> "); // Prompt for next command
        }
    }
}

void printCliHelp()
{
    Serial.println("Available commands:");
    Serial.println("  get <parameter>       - Get a setting value (e.g., 'get pid.roll.kp')");
    Serial.println("  set <parameter> <value> - Set a new value (e.g., 'set pid.roll.kp 0.95')");
    Serial.println("  debug                 - Print a snapshot of runtime flight data.");
    Serial.println("  save                  - Save current settings to non-volatile memory.");
    Serial.println("  reset                 - Reset all settings to their default values and save.");
    Serial.println("  reboot                - Reboot the flight controller.");
    Serial.println("  help                  - Show this help message.");
    Serial.println("  exit                  - Deactivate the CLI.");
    Serial.print("CLI> ");
}

void executeCliCommand(String command)
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
    else if (commandName.equals("debug"))
    {
        printFlightStatus();
    }
    else if (commandName.equals("save"))
    {
        saveSettings();
    }
    else if (commandName.equals("reset"))
    {
        settings = FlightControllerSettings(); // Overwrite current settings with defaults
        saveSettings();                        // Persist the defaults to flash
        Serial.println("INFO: All settings have been reset to their default values and saved.");
    }
    else if (commandName.equals("reboot"))
    {
        Serial.println("Rebooting...");
        delay(100); // Allow serial message to send
        ESP.restart();
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
}

// Basic implementation for 'get'. A more robust solution would use nested parsing.
void handleGetCommand(String args)
{
    if (args.equals("pid.roll.kp"))
        Serial.println(settings.pidRoll.kp);
    else if (args.equals("pid.roll.ki"))
        Serial.println(settings.pidRoll.ki);
    else if (args.equals("pid.roll.kd"))
        Serial.println(settings.pidRoll.kd);
    else if (args.equals("pid.pitch.kp"))
        Serial.println(settings.pidPitch.kp);
    else if (args.equals("pid.pitch.ki"))
        Serial.println(settings.pidPitch.ki);
    else if (args.equals("pid.pitch.kd"))
        Serial.println(settings.pidPitch.kd);
    else if (args.equals("pid.yaw.kp"))
        Serial.println(settings.pidYaw.kp);
    else if (args.equals("pid.yaw.ki"))
        Serial.println(settings.pidYaw.ki);
    else if (args.equals("pid.yaw.kd"))
        Serial.println(settings.pidYaw.kd);
    else if (args.equals("filter.gain"))
        Serial.println(settings.filter.complementaryFilterGain);
    else
    {
        Serial.println("Unknown or unsupported parameter for 'get'.");
    }
}

// Basic implementation for 'set'. A more robust solution would use nested parsing.
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
    float value = valueStr.toFloat();

    if (param.equals("pid.roll.kp"))
        settings.pidRoll.kp = value;
    else if (param.equals("pid.roll.ki"))
        settings.pidRoll.ki = value;
    else if (param.equals("pid.roll.kd"))
        settings.pidRoll.kd = value;
    else if (param.equals("pid.pitch.kp"))
        settings.pidPitch.kp = value;
    else if (param.equals("pid.pitch.ki"))
        settings.pidPitch.ki = value;
    else if (param.equals("pid.pitch.kd"))
        settings.pidPitch.kd = value;
    else if (param.equals("pid.yaw.kp"))
        settings.pidYaw.kp = value;
    else if (param.equals("pid.yaw.ki"))
        settings.pidYaw.ki = value;
    else if (param.equals("pid.yaw.kd"))
        settings.pidYaw.kd = value;
    else if (param.equals("filter.gain"))
        settings.filter.complementaryFilterGain = value;
    else
    {
        Serial.println("Unknown or unsupported parameter for 'set'.");
        return; // Don't print the confirmation message
    }

    Serial.print("Set ");
    Serial.print(param);
    Serial.print(" to ");
    Serial.println(value, 4); // Print float with 4 decimal places
}
