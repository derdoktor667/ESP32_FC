// pid_controller.cpp
//
// This file implements the PIDController class, providing the core logic for
// Proportional-Integral-Derivative (PID) control. It calculates the output
// based on the error between a setpoint and the current value, considering
// proportional, integral, and derivative terms.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "src/utils/pid/pid_controller.h"

// Define a constant for converting microseconds to seconds
static constexpr float MICROSECONDS_TO_SECONDS = 1000000.0f;

PIDController::PIDController(float p, float i, float d)
    : Kp(p), Ki(i), Kd(d), _integralSum(0), _previousError(0), _lastPidTime(0) {}

float PIDController::calculate(float setpoint, float current_value, float integral_limit)
{
  unsigned long current_time = micros();
  float dt = (current_time - _lastPidTime) / MICROSECONDS_TO_SECONDS; // Convert to seconds
  _lastPidTime = current_time;

  float error = setpoint - current_value;

  // Proportional term
  float p_term = Kp * error;

  // Integral term
  _integralSum += error * dt;
  // Limit integral sum to prevent wind-up
  if (_integralSum > integral_limit)
    _integralSum = integral_limit;
  else if (_integralSum < -integral_limit)
    _integralSum = -integral_limit;
  float i_term = Ki * _integralSum;

  // Derivative term
  // Avoid division by zero if dt is very small
  float d_term = 0.0f;
  if (dt > 0)
  {
    d_term = Kd * (error - _previousError) / dt;
  }
  _previousError = error;

  return p_term + i_term + d_term;
}
