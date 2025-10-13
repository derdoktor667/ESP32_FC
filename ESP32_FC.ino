#include "src/main/flight_controller.h"
#include "src/main/CommunicationManager.h" // New include
#include "src/config/config.h"

// Global instances of the core components
FlightController* fc; // Change to pointer
CommunicationManager* comms; // New global pointer

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Instantiate after Serial.begin()
    fc = new FlightController();
    comms = new CommunicationManager(fc); // Pass fc to comms constructor

    fc->setCommunicationManager(comms); // New setter in FlightController to set _comms

    fc->initialize();
    comms->begin(); // Initialize comms

    Serial.println(" ");
    Serial.println("\n--- ESP32 Flight Controller Ready ---");
}

void loop()
{
    fc->runLoop();
    comms->update(fc->state); // Pass current state to comms
}
