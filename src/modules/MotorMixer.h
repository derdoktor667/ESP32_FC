#ifndef MOTOR_MIXER_MODULE_H
#define MOTOR_MIXER_MODULE_H

#include "../FlightState.h"
#include <DShotRMT.h>

// DShot protocol fixed throttle values
static constexpr int DSHOT_MIN_THROTTLE = 48;
static constexpr int DSHOT_MAX_THROTTLE = 2047;

// Mixes PID outputs with throttle and sends commands to the motors.
class MotorMixer
{
public:
    MotorMixer(DShotRMT &motor1, DShotRMT &motor2, DShotRMT &motor3, DShotRMT &motor4, const FlightControllerSettings &settings);

    // Initializes the motors.
    void begin();

    // Applies the calculated outputs to the motors.
    // Reads throttle and PID outputs from the FlightState and sends DShot commands.
    // - state: The current, read-only flight state.
    void apply(FlightState &state);

private:
    DShotRMT &_motor1; // Front-Right
    DShotRMT &_motor2; // Front-Left
    DShotRMT &_motor3; // Rear-Right
    DShotRMT &_motor4; // Rear-Left
    const FlightControllerSettings &_settings;
};

#endif // MOTOR_MIXER_MODULE_H
