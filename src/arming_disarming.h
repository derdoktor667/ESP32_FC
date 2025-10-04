#ifndef ARMING_DISARMING_H
#define ARMING_DISARMING_H

#include <Arduino.h>

extern bool armed;          // External arming status
extern bool failsafeActive; // External failsafe status

void handleSafetySwitches();

#endif
