#include "src/utils/pid/pid_controller.h"

PIDController::PIDController(float p, float i, float d)
    : Kp(p), Ki(i), Kd(d), _integralSum(0), _previousError(0), _lastPidTime(0) {}

float PIDController::calculate(float setpoint, float current_value, float integral_limit)
{
  unsigned long current_time = micros();
  float dt = (current_time - _lastPidTime) / 1000000.0; // Convert to seconds
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
  float d_term = Kd * (error - _previousError) / dt;
  _previousError = error;

  return p_term + i_term + d_term;
}
