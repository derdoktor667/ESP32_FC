#include "src/flight_controller.h"
#include "src/config.h"

// Global instance of the main flight controller class
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