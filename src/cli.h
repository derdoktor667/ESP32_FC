#ifndef CLI_H
#define CLI_H

#include "FlightState.h"

// Enum to represent commands that the CLI might request the FlightController to perform.
enum class CliCommand
{
    NONE,
    CALIBRATE_IMU,
    // Add other commands here as needed
};

// Updated to return a CliCommand, indicating if a special action is requested.
CliCommand handleSerialCli(const FlightState &state);

// Helper functions for protocol string conversion
String getReceiverProtocolString(ReceiverProtocol protocol);
String getImuProtocolString(ImuProtocol protocol);
String getFlightControlInputString(FlightControlInput input);

#endif