#ifndef SERIAL_LOGGER_H
#define SERIAL_LOGGER_H

#include <Arduino.h>
#include "FlightState.h"

void printFlightStatus(const FlightState &state);

#endif
