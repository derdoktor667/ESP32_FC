#include "src/flight_controller.h"

// Global instance of the main flight controller class
FlightController fc;

void setup()
{
    Serial.begin(115200);
    fc.initialize();
    Serial.println("\n--- ESP32 Flight Controller Ready ---");
}

void loop()
{
    fc.runLoop();
}