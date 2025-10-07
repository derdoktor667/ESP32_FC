#include "PidProcessor.h"
#include "../config.h"

// Constructor: Initializes the PID controllers with gains from the settings.
PidProcessor::PidProcessor(const FlightControllerSettings &settings)
    : _settings(settings),
      _pidRoll(settings.pidRoll.kp / (float)PID_SCALE_FACTOR, settings.pidRoll.ki / (float)PID_SCALE_FACTOR, settings.pidRoll.kd / (float)PID_SCALE_FACTOR),
      _pidPitch(settings.pidPitch.kp / (float)PID_SCALE_FACTOR, settings.pidPitch.ki / (float)PID_SCALE_FACTOR, settings.pidPitch.kd / (float)PID_SCALE_FACTOR),
      _pidYaw(settings.pidYaw.kp / (float)PID_SCALE_FACTOR, settings.pidYaw.ki / (float)PID_SCALE_FACTOR, settings.pidYaw.kd / (float)PID_SCALE_FACTOR)
{
}

// Calculates the PID outputs for roll, pitch, and yaw.
void PidProcessor::update(FlightState &state)
{
    float roll_target = 0.0f;
    float pitch_target = 0.0f;

    if (state.currentFlightMode == ANGLE_MODE)
    {
        // In Angle mode, the setpoint is an angle, and the PID input is the angle error.
        roll_target = state.setpoints.roll;
        pitch_target = state.setpoints.pitch;
    }
    else // ACRO_MODE
    {
        // In Acro mode, the setpoint is a rate, and the PID input is the rate error.
        // The PID controller will try to make the drone's rotation rate match the stick input.
        roll_target = state.gyroRates.roll;
        pitch_target = state.gyroRates.pitch;
    }

    // Calculate PID outputs
    state.pidOutput.roll = _pidRoll.calculate(state.setpoints.roll, roll_target, _settings.pidIntegralLimit);
    state.pidOutput.pitch = _pidPitch.calculate(state.setpoints.pitch, pitch_target, _settings.pidIntegralLimit);
    state.pidOutput.yaw = _pidYaw.calculate(state.setpoints.yaw, state.gyroRates.yaw, _settings.pidIntegralLimit);
}
