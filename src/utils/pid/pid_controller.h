#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include "src/config/config.h" // For PID_INTEGRAL_LIMIT
#include "src/config/settings.h"

struct PIDController
{
  float Kp, Ki, Kd;
  float integral_sum;
  float previous_error;
  unsigned long last_pid_time;

  PIDController(float p, float i, float d);
  float calculate(float setpoint, float current_value, float integral_limit);
};

#endif
