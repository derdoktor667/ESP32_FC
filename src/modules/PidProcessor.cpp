// PidProcessor.cpp
//
// This file implements the PidProcessor class, which manages and executes the
// PID (Proportional-Integral-Derivative) control loops for the drone's
// roll, pitch, and yaw axes. It calculates the necessary corrections based on
// desired setpoints and current flight state.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "src/modules/PidProcessor.h"
#include "src/config/config.h" // Required for PID_SCALE_FACTOR

// Constructor: Initializes the PID controllers for Roll, Pitch, and Yaw axes.
// PID gains (Kp, Ki, Kd) are loaded from the global settings, scaled by PID_SCALE_FACTOR
// to convert integer settings to floating-point PID gains.
PidProcessor::PidProcessor(const PidSettings &pidSettings)
    : _pidSettings(pidSettings)
{
    _initializePidControllers();
}

// Calculates the PID outputs for roll, pitch, and yaw axes based on current flight mode and state.
// The calculated PID outputs are stored in the FlightState object.
void PidProcessor::update(FlightState &state)
{
    float roll_input_for_pid = 0.0f;
    float pitch_input_for_pid = 0.0f;

    // Determine the appropriate input for the PID controller based on the current flight mode.
    if (state.currentFlightMode == ANGLE_MODE)
    {
        // In ANGLE_MODE, the setpoint (from SetpointManager) is a target angle.
        // The PID controller's input is the current angle (attitude).
        roll_input_for_pid = state.attitude.roll;
        pitch_input_for_pid = state.attitude.pitch;
    }
    else // ACRO_MODE (or any other rate-based mode)
    {
        // In ACRO_MODE, the setpoint (from SetpointManager) is a target rotation rate.
        // The PID controller's input is the current rotation rate (gyroRates).
        roll_input_for_pid = state.gyroRates.roll;
        pitch_input_for_pid = state.gyroRates.pitch;
    }

    // Calculate PID outputs for each axis.
    // The calculate method takes (setpoint, current_value, integral_limit).
    state.pidOutput.roll = _pidRoll->calculate(state.setpoints.roll, roll_input_for_pid, _pidSettings.integralLimit);
    state.pidOutput.pitch = _pidPitch->calculate(state.setpoints.pitch, pitch_input_for_pid, _pidSettings.integralLimit);
    // Yaw control is typically always rate-based, so its input is always the gyro yaw rate.
    state.pidOutput.yaw = _pidYaw->calculate(state.setpoints.yaw, state.gyroRates.yaw, _pidSettings.integralLimit);
}

void PidProcessor::_initializePidControllers()
{
    _pidRoll = std::make_unique<PIDController>(_pidSettings.roll.kp / (float)PID_SCALE_FACTOR, _pidSettings.roll.ki / (float)PID_SCALE_FACTOR, _pidSettings.roll.kd / (float)PID_SCALE_FACTOR);
    _pidPitch = std::make_unique<PIDController>(_pidSettings.pitch.kp / (float)PID_SCALE_FACTOR, _pidSettings.pitch.ki / (float)PID_SCALE_FACTOR, _pidSettings.pitch.kd / (float)PID_SCALE_FACTOR);
    _pidYaw = std::make_unique<PIDController>(_pidSettings.yaw.kp / (float)PID_SCALE_FACTOR, _pidSettings.yaw.ki / (float)PID_SCALE_FACTOR, _pidSettings.yaw.kd / (float)PID_SCALE_FACTOR);
}
