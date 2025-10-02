#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <DShotRMT.h>
#include "config.h"

extern DShotRMT motor1, motor2, motor3, motor4; // External motor objects

void setupMotors();
void sendMotorCommands(int throttle, float pid_output_roll, float pid_output_pitch, float pid_output_yaw, bool armed);

#endif
