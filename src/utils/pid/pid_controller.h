#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>


class PIDController
{
public:
  // Proportional, Integral, and Derivative gains
  float Kp, Ki, Kd;

  // Constructor: Initializes PID gains.
  PIDController(float p, float i, float d);

  // Calculates the PID output.
  float calculate(float setpoint, float current_value, float integral_limit);

private:
  // Accumulator for the integral term.
  float _integralSum;
  // Stores the error from the previous iteration for derivative calculation.
  float _previousError;
  // Timestamp of the last PID calculation for delta time calculation.
  unsigned long _lastPidTime;
};

#endif
