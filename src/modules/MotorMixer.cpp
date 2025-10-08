#include "MotorMixer.h"
#include "../config.h"
#include <Arduino.h>

// Constructor: Initializes with references to the four motor objects.
MotorMixer::MotorMixer(DShotRMT &motor1, DShotRMT &motor2, DShotRMT &motor3, DShotRMT &motor4, const FlightControllerSettings &settings)
    : _motor1(motor1), _motor2(motor2), _motor3(motor3), _motor4(motor4), _settings(settings)
{
}

// Initializes all motors.
void MotorMixer::begin()
{
    _motor1.begin();
    _motor2.begin();
    _motor3.begin();
    _motor4.begin();
    Serial.println("DShot motors initialized.");
}

// Applies the final calculated throttle values to the motors.
void MotorMixer::apply(FlightState &state)
{
    if (!state.isArmed || state.isFailsafeActive)
    {
        // If not armed or if failsafe is active, command motors to stop.
        _motor1.sendThrottle(0);
        _motor2.sendThrottle(0);
        _motor3.sendThrottle(0);
        _motor4.sendThrottle(0);
        return;
    }

    // --- Motor Mixing (X-quad configuration) ---
    float m1 = state.throttle - state.pidOutput.roll + state.pidOutput.pitch + state.pidOutput.yaw; // Front Right
    float m2 = state.throttle + state.pidOutput.roll + state.pidOutput.pitch - state.pidOutput.yaw; // Front Left
    float m3 = state.throttle - state.pidOutput.roll - state.pidOutput.pitch - state.pidOutput.yaw; // Rear Right
    float m4 = state.throttle + state.pidOutput.roll - state.pidOutput.pitch + state.pidOutput.yaw; // Rear Left

    // Calculate the minimum throttle value based on user setting, ensuring it's at least DSHOT_MIN_THROTTLE
    float minActiveThrottle = DSHOT_MAX_THROTTLE * (_settings.motorIdleSpeedPercent / 100.0f);
    int effectiveMinThrottle = max((int)DSHOT_MIN_THROTTLE, (int)minActiveThrottle);

    // Constrain motor values to the allowed DShot range and store them in the state
    state.motorOutputs[0] = constrain(m1, effectiveMinThrottle, DSHOT_MAX_THROTTLE);
    state.motorOutputs[1] = constrain(m2, effectiveMinThrottle, DSHOT_MAX_THROTTLE);
    state.motorOutputs[2] = constrain(m3, effectiveMinThrottle, DSHOT_MAX_THROTTLE);
    state.motorOutputs[3] = constrain(m4, effectiveMinThrottle, DSHOT_MAX_THROTTLE);

    // Send the final commands to the motors
    _motor1.sendThrottle(state.motorOutputs[0]);
    _motor2.sendThrottle(state.motorOutputs[1]);
    _motor3.sendThrottle(state.motorOutputs[2]);
    _motor4.sendThrottle(state.motorOutputs[3]);
}
