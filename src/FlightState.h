#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include "config.h"

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

    // --- Receiver & Setpoints ---
    uint16_t receiverChannels[16]; // Raw values from the receiver (e.g., 1000-2000)
    Vector3f setpoints;            // Target values for roll, pitch, yaw (can be angle or rate)

    // --- Safety & Status ---
    bool isArmed = false;
    bool isFailsafeActive = false;
    FlightMode currentFlightMode = ACRO_MODE;

    // --- Control & Output ---
    float throttle = 0.0f;         // Base throttle from the receiver, mapped to output range
    Vector3f pidOutput;            // The output from the PID controllers for each axis
    float motorOutputs[4] = {0.0f}; // Final throttle values for each motor
};

#endif // FLIGHT_STATE_H
