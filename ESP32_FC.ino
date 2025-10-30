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
FlightController *fc;
CommunicationManager *comms;

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.flush(); // Clear any pending incoming serial data

    // Instantiate after Serial.begin()
    fc = new FlightController();
    comms = new CommunicationManager(fc);

    fc->initialize();
    comms->initializeCommunication();
}

void loop()
{
    fc->runLoop();
    comms->processCommunication();
}
