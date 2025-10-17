// PidProcessor.h
//
// This file defines the PidProcessor class, which manages and executes the
// PID (Proportional-Integral-Derivative) control loops for the drone's
// roll, pitch, and yaw axes. It takes desired setpoints and current attitude
// to calculate necessary corrections.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef PID_PROCESSOR_MODULE_H
#define PID_PROCESSOR_MODULE_H

#include "src/config/FlightState.h"
#include "src/utils/pid/pid_controller.h"
#include <memory> // Required for std::unique_ptr

// Processes the PID control loops for all axes (Roll, Pitch, Yaw).
// This module takes the desired setpoints (from SetpointManager) and the current
// attitude (from AttitudeEstimator), calculates the necessary PID corrections
// to reach the setpoints, and stores these corrections in the FlightState.
class PidProcessor
{
public:
    // Constructor: Initializes the PID controllers for Roll, Pitch, and Yaw.
    // @param settings Reference to the global flight controller settings,
    //                 which contain the PID gains (Kp, Ki, Kd).
    PidProcessor(const FlightControllerSettings &settings);
    // Destructor is no longer explicitly needed as std::unique_ptr handles cleanup

    // Performs one cycle of PID calculations for all axes.
    // It reads the current attitude and setpoints from the FlightState,
    // computes the PID outputs, and writes them back into the FlightState.
    // @param state Reference to the current FlightState, used for input and updated with PID output.
    void update(FlightState &state);

private:
    std::unique_ptr<PIDController> _pidRoll;  // PID controller instance for the Roll axis
    std::unique_ptr<PIDController> _pidPitch; // PID controller instance for the Pitch axis
    std::unique_ptr<PIDController> _pidYaw;   // PID controller instance for the Yaw axis
    const FlightControllerSettings &_settings; // Reference to global flight controller settings

    // Private helper method for PID controller initialization
    void _initializePidControllers();
};

#endif // PID_PROCESSOR_MODULE_H
