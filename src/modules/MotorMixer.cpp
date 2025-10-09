#include "MotorMixer.h"
#include "../config.h"
#include <Arduino.h>

// Constructor: Initializes the MotorMixer with references to the four DShot motor objects and settings.
MotorMixer::MotorMixer(DShotRMT &motor1, DShotRMT &motor2, DShotRMT &motor3, DShotRMT &motor4, const FlightControllerSettings &settings)
    : _motor1(motor1), _motor2(motor2), _motor3(motor3), _motor4(motor4), _settings(settings)
{
}

// Initializes all DShot motors by calling their individual begin() methods.
void MotorMixer::begin()
{
    _motor1.begin();
    _motor2.begin();
    _motor3.begin();
    _motor4.begin();
    Serial.println("DShot motors initialized.");
}

// Applies the final calculated throttle values to the motors based on flight state.
void MotorMixer::apply(FlightState &state)
{
    // Safety check: If not armed or if failsafe is active, all motors must be stopped immediately.
    if (!state.isArmed || state.isFailsafeActive)
    {
        _motor1.sendThrottle(0);
        _motor2.sendThrottle(0);
        _motor3.sendThrottle(0);
        _motor4.sendThrottle(0);
        return; // Exit early as no further motor commands should be sent.
    }

    // --- Motor Mixing for X-quad configuration ---
    // The PID outputs (roll, pitch, yaw) are added/subtracted from the base throttle
    // to achieve the desired attitude changes.
    // m1: Front Right motor
    // m2: Front Left motor
    // m3: Rear Right motor
    // m4: Rear Left motor
    float m1 = state.throttle - state.pidOutput.roll + state.pidOutput.pitch + state.pidOutput.yaw;
    float m2 = state.throttle + state.pidOutput.roll + state.pidOutput.pitch - state.pidOutput.yaw;
    float m3 = state.throttle - state.pidOutput.roll - state.pidOutput.pitch - state.pidOutput.yaw;
    float m4 = state.throttle + state.pidOutput.roll - state.pidOutput.pitch + state.pidOutput.yaw;

    // Calculate the effective minimum throttle value.
    // This ensures motors spin at a minimum speed when armed (idle speed)
    // but never below the absolute DSHOT_MIN_THROTTLE.
    float minActiveThrottle = DSHOT_MAX_THROTTLE * (_settings.motorIdleSpeedPercent / 100.0f);
    int effectiveMinThrottle = max((int)DSHOT_MIN_THROTTLE, (int)minActiveThrottle);

    // Constrain each motor's calculated value to be within the valid DShot range
    // (from effectiveMinThrottle to DSHOT_MAX_THROTTLE).
    // Store these final values in the FlightState for logging/debugging.
    state.motorOutputs[0] = constrain(m1, effectiveMinThrottle, DSHOT_MAX_THROTTLE);
    state.motorOutputs[1] = constrain(m2, effectiveMinThrottle, DSHOT_MAX_THROTTLE);
    state.motorOutputs[2] = constrain(m3, effectiveMinThrottle, DSHOT_MAX_THROTTLE);
    state.motorOutputs[3] = constrain(m4, effectiveMinThrottle, DSHOT_MAX_THROTTLE);

    // Send the final, constrained throttle commands to the DShot ESCs.
    _motor1.sendThrottle(state.motorOutputs[0]);
    _motor2.sendThrottle(state.motorOutputs[1]);
    _motor3.sendThrottle(state.motorOutputs[2]);
    _motor4.sendThrottle(state.motorOutputs[3]);
}
