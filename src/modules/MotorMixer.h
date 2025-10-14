// MotorMixer.h
//
// This file defines the MotorMixer class, which is responsible for mixing
// PID outputs with the main throttle input to calculate individual motor
// commands. It then sends these commands to the ESCs via the DShotRMT protocol.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef MOTOR_MIXER_MODULE_H
#define MOTOR_MIXER_MODULE_H

#include "src/config/FlightState.h"
#include <DShotRMT.h>

// Mixes PID outputs with throttle and sends commands to the motors.
// This module takes the desired throttle and PID corrections from the FlightState,
// calculates individual motor commands, and sends them via DShotRMT.
class MotorMixer
{
public:
    // Constructor: Initializes the MotorMixer with references to the DShot motor instances and settings.
    MotorMixer(DShotRMT *motor1, DShotRMT *motor2, DShotRMT *motor3, DShotRMT *motor4, const FlightControllerSettings &settings);

    // Initializes the DShot motors.
    // This typically involves setting up the RMT channels.
    void begin();

    // Applies the calculated outputs to the motors.
    // Reads the current throttle and PID outputs from the FlightState,
    // performs the motor mixing algorithm, and sends the resulting DShot commands to each motor.
    void apply(FlightState &state);

private:
    DShotRMT *_motor1; // Pointer to the Front-Right DShot motor driver
    DShotRMT *_motor2; // Pointer to the Front-Left DShot motor driver
    DShotRMT *_motor3; // Pointer to the Rear-Right DShot motor driver
    DShotRMT *_motor4; // Pointer to the Rear-Left DShot motor driver
    const FlightControllerSettings &_settings; // Reference to global flight controller settings
};

#endif // MOTOR_MIXER_MODULE_H
