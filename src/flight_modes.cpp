#include "flight_modes.h"

// External IBUS object
extern FlyskyIBUS ibusReceiver;

// Flight Modes
FlightMode current_flight_mode = ACRO_MODE;

void handleFlightModeSelection()
{
  int flight_mode_channel_value = ibusReceiver.getChannel(IBUS_CHANNEL_FLIGHT_MODE);

  if (flight_mode_channel_value < 1200)
  { // Example: Switch low for Acro Mode
    current_flight_mode = ACRO_MODE;
  }
  else if (flight_mode_channel_value > 1800)
  { // Example: Switch high for Angle Mode
    current_flight_mode = ANGLE_MODE;
  } // Middle position could be Horizon Mode if a 3-pos switch is used
}
