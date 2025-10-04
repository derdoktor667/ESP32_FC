#ifndef SERIAL_LOGGER_H
#define SERIAL_LOGGER_H

#include <Arduino.h>
#include <ESP32_MPU6050.h>
#include <FlyskyIBUS.h>
#include "config.h"
#include "attitude_estimator.h"
#include "arming_disarming.h"
#include "flight_modes.h"

extern ESP32_MPU6050 imuSensor; // External MPU object
extern FlyskyIBUS ibusReceiver;   // External IBUS object

extern float roll, pitch, yaw;                      // External attitude variables
extern float target_roll, target_pitch, target_yaw; // External target setpoints
extern bool armed;                                  // External arming status
extern bool failsafeActive;                         // External failsafe status
extern int arming_channel_value;                    // External arming channel value
extern FlightMode current_flight_mode;              // External current flight mode

void printFlightStatus();

#endif
