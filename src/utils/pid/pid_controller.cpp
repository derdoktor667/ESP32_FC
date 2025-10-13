#include "src/utils/pid/pid_controller.h"

PIDController::PIDController(float p, float i, float d)
    : Kp(p), Ki(i), Kd(d), integral_sum(0), previous_error(0), last_pid_time(0) {}

float PIDController::calculate(float setpoint, float current_value, float integral_limit)
{
  unsigned long current_time = micros();
  float dt = (current_time - last_pid_time) / 1000000.0; // Convert to seconds
  last_pid_time = current_time;

  float error = setpoint - current_value;

  // Proportional term
  float p_term = Kp * error;

  // Integral term
  integral_sum += error * dt;
  // Limit integral sum to prevent wind-up
  if (integral_sum > integral_limit)
    integral_sum = integral_limit;
  else if (integral_sum < -integral_limit)
    integral_sum = -integral_limit;
  float i_term = Ki * integral_sum;

  // Derivative term
  float d_term = Kd * (error - previous_error) / dt;
  previous_error = error;

  return p_term + i_term + d_term;
}
