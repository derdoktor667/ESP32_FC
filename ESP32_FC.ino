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

void testCrc() {
    Serial.println("--- CRC Test ---");
    uint8_t crc = 0;
    uint8_t bytes[] = {0x3E, 0x00, 0x00, 0x00, 0xDA, 0x07}; // >, flags, size_low, size_high, command_low, command_high for GET_VERSION

    for (int i = 0; i < sizeof(bytes); ++i) {
        crc = crc8_dvb_s2(crc, bytes[i]);
        Serial.print("Byte: 0x");
        Serial.print(bytes[i], HEX);
        Serial.print(", Current CRC: 0x");
        Serial.println(crc, HEX);
    }
    Serial.print("Final CRC for GET_VERSION: 0x");
    Serial.println(crc, HEX);
    Serial.println("----------------");
}

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.flush(); // Clear any pending incoming serial data

    // Instantiate after Serial.begin()
    fc = new FlightController();
    comms = new CommunicationManager(fc);



    fc->initialize();
    comms->initializeCommunication();

    testCrc(); // Call CRC test function

    Serial.println(); // Print a blank line for readability
    Serial.print("--- ESP32 Flight Controller Ready (v");
    Serial.print(FIRMWARE_VERSION);
    Serial.println(") ---");
}

void loop()
{
    fc->runLoop();
    comms->processCommunication();
}
