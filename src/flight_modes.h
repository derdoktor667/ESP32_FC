#ifndef FLIGHT_MODES_H
#define FLIGHT_MODES_H

#include <Arduino.h>
#include <FlyskyIBUS.h>
#include "config.h"

extern FlyskyIBUS ibus; // External IBUS object

// Flight Modes
enum FlightMode
{
  ACRO_MODE,
  ANGLE_MODE,
  // Add more modes as needed
};

extern FlightMode current_flight_mode; // External current flight mode

void handleFlightModeSelection();

#endif
