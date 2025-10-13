#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include "src/config/config.h"

// A simple struct to hold 3-axis float data (e.g., for attitude or rates)
struct Vector3f
{
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

// Holds the complete, dynamic state of the flight controller at any given moment.
//
// This struct is passed to each processing module, which reads from and writes to it.
// It acts as the single source of truth for the drone's state, eliminating global variables
// and making the data flow explicit.
struct FlightState
{
    // --- Sensor & Attitude Data ---
    Vector3f attitude;      // Estimated orientation (roll, pitch, yaw) in degrees
    Vector3f gyroRates;     // Calibrated gyroscope readings (roll, pitch, yaw) in deg/s
    Vector3f accelReadings; // Calibrated accelerometer readings (X, Y, Z) in g
    float imuTemperature = 0.0f; // IMU temperature in Celsius

    // --- Receiver & Setpoints ---
    uint16_t receiverChannels[RECEIVER_CHANNEL_COUNT]; // Raw values from the receiver (e.g., 1000-2000)
    Vector3f setpoints;            // Target values for roll, pitch, yaw (can be angle or rate)
    int16_t rssi = 0;              // Receiver signal strength indicator

    // --- Safety & Status ---
    bool isArmed = false;
    bool isFailsafeActive = false;
    FlightMode currentFlightMode = ACRO_MODE;
    uint32_t armedTimeS = 0;       // Time in seconds since arming

    // --- Control & Output ---
    float throttle = 0.0f;         // Base throttle from the receiver, mapped to output range
    Vector3f pidOutput;            // The output from the PID controllers for each axis
    float motorOutputs[4] = {0.0f}; // Final throttle values for each motor

    // --- System Metrics ---
    uint32_t loopTimeUs = 0;       // Duration of the main flight loop in microseconds
    float voltage = 0.0f;          // Battery voltage
    float current = 0.0f;          // Current draw
    float cpuLoad = 0.0f;          // CPU load percentage
};

#endif // FLIGHT_STATE_H
