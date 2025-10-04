#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <ESP32_MPU6050.h>
#include <FlyskyIBUS.h>
#include <DShotRMT.h>
#include "pid_controller.h"
#include "flight_modes.h"

// Declare global objects to be accessible by other modules
extern ESP32_MPU6050 imuSensor;
extern FlyskyIBUS ibusReceiver;
extern PIDController pid_roll, pid_pitch, pid_yaw;

// Declare global motor objects
extern DShotRMT motorFrontRight, motorFrontLeft, motorRearRight, motorRearLeft;

// Declare global state variables
extern float target_roll, target_pitch, target_yaw;
extern float roll, pitch, yaw;
extern bool armed;
extern bool failsafeActive;
extern int arming_channel_value;
extern unsigned long last_print_time;
extern unsigned long last_attitude_update_time;
extern float gyro_offset_x, gyro_offset_y, gyro_offset_z;
extern float acc_offset_x, acc_offset_y, acc_offset_z;

void initializeFlightController();
void runFlightLoop();

#endif