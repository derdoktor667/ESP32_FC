#ifndef PID_PROCESSOR_MODULE_H
#define PID_PROCESSOR_MODULE_H

#include "../FlightState.h"
#include "../pid_controller.h"

// Processes the PID control loops for all axes.
//
// This class takes the desired setpoints and the current attitude,
// calculates the necessary PID corrections, and stores them in the flight state.
class PidProcessor
{
public:
    PidProcessor(const FlightControllerSettings &settings);

    // Performs one cycle of PID calculations for all axes.
    // - state: The current flight state, used for input and updated with PID output.
    void update(FlightState &state);

private:
    PIDController _pidRoll;
    PIDController _pidPitch;
    PIDController _pidYaw;
    const FlightControllerSettings &_settings; // Reference to global settings
};

#endif // PID_PROCESSOR_MODULE_H
