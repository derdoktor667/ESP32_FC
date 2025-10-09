#include "serial_logger.h"
#include "flight_controller.h"

// This function is now responsible for printing the flight status based on the provided state.
void printFlightStatus(const FlightState &state)
{
    if (!settings.enableLogging)
    {
        return; // Do not print if logging is disabled globally
    }

    // Manually construct the JSON string
    Serial.print("{");

    // Attitude
    Serial.print("\"attitude\":{");
    Serial.print("\"roll\":");
    Serial.print(state.attitude.roll, 2);
    Serial.print(",\"pitch\":");
    Serial.print(state.attitude.pitch, 2);
    Serial.print(",\"yaw\":");
    Serial.print(state.attitude.yaw, 2);
    Serial.print("},");

    // Setpoints
    Serial.print("\"setpoints\":{");
    Serial.print("\"roll\":");
    Serial.print(state.setpoints.roll, 2);
    Serial.print(",\"pitch\":");
    Serial.print(state.setpoints.pitch, 2);
    Serial.print(",\"yaw\":");
    Serial.print(state.setpoints.yaw, 2);
    Serial.print(",\"throttle\":");
    Serial.print(state.setpoints.throttle, 2);
    Serial.print("},");

    // Status
    Serial.print("\"status\":{");
    Serial.print("\"armed\":");
    Serial.print(state.isArmed ? "true" : "false");
    Serial.print(",\"failsafe\":");
    Serial.print(state.isFailsafeActive ? "true" : "false");
    Serial.print(",\"mode\":\"");
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
    Serial.print("\"},");

    // Receiver Channels
    Serial.print("\"receiver\":[");
    for (int i = 0; i < RECEIVER_CHANNEL_COUNT; i++)
    {
        Serial.print(state.receiverChannels[i]);
        if (i < RECEIVER_CHANNEL_COUNT - 1)
        {
            Serial.print(",");
        }
    }
    Serial.print("],");

    // Loop Time
    Serial.print("\"loop_time_us\":");
    Serial.print(state.loopTimeUs);
    Serial.print(",");

    // Motor Output
    Serial.print("\"motor_output\":[");
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        Serial.print(state.motorOutputs[i]);
        if (i < NUM_MOTORS - 1)
        {
            Serial.print(",");
        }
    }
    Serial.print("],");

    // IMU Temperature
    Serial.print("\"imu_temp\":");
    Serial.print(state.imuTemperature, 2);
    Serial.print(",");

    // Voltage and Current (if available)
    Serial.print("\"voltage\":");
    Serial.print(state.voltage, 2);
    Serial.print(",\"current\":");
    Serial.print(state.current, 2);
    Serial.print(",");

    // RSSI
    Serial.print("\"rssi\":");
    Serial.print(state.rssi);
    Serial.print(",");

    // Armed Time
    Serial.print("\"armed_time_s\":");
    Serial.print(state.armedTimeS);
    Serial.print(",");

    // CPU Load
    Serial.print("\"cpu_load\":");
    Serial.print(state.cpuLoad, 2);
    Serial.print(",");

    // Warnings (empty array for now, can be populated later)
    Serial.print("\"warnings\":[],");

    // Errors (empty array for now, can be populated later)
    Serial.print("\"errors\":[]");

    Serial.println("}");
}