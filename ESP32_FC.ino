// ESP32_FC.ino
//
// Main entry point for the ESP32 Flight Controller firmware.
// This file orchestrates the creation and execution of the FlightController
// and CommunicationManager, setting up the core loop for drone operation.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "src/main/flight_controller.h"
#include "src/main/CommunicationManager.h"
#include "src/config/config.h"

// Global instances of the core components
FlightController* fc;
CommunicationManager* comms;

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Instantiate after Serial.begin()
    fc = new FlightController();
    comms = new CommunicationManager(fc);

    fc->setCommunicationManager(comms);

    fc->initialize();
    comms->begin();

    Serial.println(); // Print a blank line for readability
    Serial.print("--- ESP32 Flight Controller Ready (v");
    Serial.print(FIRMWARE_VERSION);
    Serial.println(") ---");
}

void loop()
{
    fc->runLoop();
    comms->update(fc->state); // Pass current state to comms
}
