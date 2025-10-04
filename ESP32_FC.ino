#include <Arduino.h>
#include "src/flight_controller.h"

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  initializeFlightController();
}

void loop()
{
  runFlightLoop();
}
