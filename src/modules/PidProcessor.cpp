#include "src/modules/PidProcessor.h"
#include "src/config/config.h"

// Constructor: Initializes the PID controllers for Roll, Pitch, and Yaw axes.
// PID gains (Kp, Ki, Kd) are loaded from the global settings, scaled by PID_SCALE_FACTOR.
PidProcessor::PidProcessor(const FlightControllerSettings &settings)
    : _settings(settings),
      _pidRoll(settings.pidRoll.kp / (float)PID_SCALE_FACTOR, settings.pidRoll.ki / (float)PID_SCALE_FACTOR, settings.pidRoll.kd / (float)PID_SCALE_FACTOR),
      _pidPitch(settings.pidPitch.kp / (float)PID_SCALE_FACTOR, settings.pidPitch.ki / (float)PID_SCALE_FACTOR, settings.pidPitch.kd / (float)PID_SCALE_FACTOR),
      _pidYaw(settings.pidYaw.kp / (float)PID_SCALE_FACTOR, settings.pidYaw.ki / (float)PID_SCALE_FACTOR, settings.pidYaw.kd / (float)PID_SCALE_FACTOR)
{
}

// Calculates the PID outputs for roll, pitch, and yaw axes based on current flight mode and state.
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
    state.pidOutput.roll = _pidRoll.calculate(state.setpoints.roll, roll_input_for_pid, _settings.pidIntegralLimit);
    state.pidOutput.pitch = _pidPitch.calculate(state.setpoints.pitch, pitch_input_for_pid, _settings.pidIntegralLimit);
    // Yaw is typically always rate-controlled, so its input is always the gyro yaw rate.
    state.pidOutput.yaw = _pidYaw.calculate(state.setpoints.yaw, state.gyroRates.yaw, _settings.pidIntegralLimit);
}
