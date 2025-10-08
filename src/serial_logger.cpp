#include "serial_logger.h"
#include "flight_controller.h"

// This function is now responsible for printing the flight status based on the provided state.
void printFlightStatus(const FlightState &state)
{
    if (!settings.enableLogging) {
        return; // Do not print if logging is disabled globally
    }
    // Example of how to print the new state-based data.
    // This is not exhaustive and can be expanded.
    Serial.print("Roll: ");
    Serial.print(state.attitude.roll);
    Serial.print("\tPitch: ");
    Serial.print(state.attitude.pitch);
    Serial.print("\tYaw: ");
    Serial.print(state.attitude.yaw);

    Serial.print("\tSetRoll: ");
    Serial.print(state.setpoints.roll);
    Serial.print("\tSetPitch: ");
    Serial.print(state.setpoints.pitch);

    Serial.print("\tArmed: ");
    Serial.print(state.isArmed ? "YES" : "NO");
    Serial.print("\tFailsafe: ");
    Serial.print(state.isFailsafeActive ? "ACTIVE" : "inactive");

    Serial.print("\tMode: ");
    switch (state.currentFlightMode)
    {
    case ACRO_MODE:
        Serial.print("ACRO");
        break;
    case ANGLE_MODE:
        Serial.print("ANGLE");
        break;
    default:
        Serial.print("UNKNOWN");
        break;
    }
    Serial.println();
}