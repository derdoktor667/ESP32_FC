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
FlightController *flightController;
CommunicationManager *communicationManager;

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.flush(); // Clear any pending incoming serial data

    // Instantiate after Serial.begin()
    flightController = new FlightController();
    communicationManager = new CommunicationManager(flightController);

    flightController->initialize();
}

void loop()
{
    flightController->runLoop();
    communicationManager->processCommunication();
}