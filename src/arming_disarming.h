#ifndef ARMING_DISARMING_H
#define ARMING_DISARMING_H

#include <Arduino.h>
#include <FlyskyIBUS.h>
#include "config.h"
#include <DShotRMT.h> // For motor control

extern FlyskyIBUS ibusReceiver;                                                 // External IBUS object
extern DShotRMT motorFrontRight, motorFrontLeft, motorRearRight, motorRearLeft; // External motor objects

extern bool armed;           // External arming status
extern bool failsafeActive;  // External failsafe status

void handleSafetySwitches();

#endif
