#ifndef ARMING_DISARMING_H
#define ARMING_DISARMING_H

#include <Arduino.h>
#include <FlyskyIBUS.h>
#include "config.h"
#include <DShotRMT.h> // For motor control

extern FlyskyIBUS ibus;                         // External IBUS object
extern DShotRMT motor1, motor2, motor3, motor4; // External motor objects

extern bool armed; // External arming status

void handleArming();

#endif
