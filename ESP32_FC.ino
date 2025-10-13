#include "src/main/flight_controller.h"
#include "src/config/config.h"

// Global instances of the core components
FlightController fc;

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    fc.initialize();
    Serial.println("\n--- ESP32 Flight Controller Ready ---");
}

void loop()
{
    fc.runLoop();
}
