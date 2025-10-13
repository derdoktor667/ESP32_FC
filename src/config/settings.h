#ifndef SETTINGS_H
#define SETTINGS_H

#include "src/config/config.h"

// Declare the global settings instance to be accessible across the project
extern FlightControllerSettings settings;

// Functions to manage persistent storage
void saveSettings();
void loadSettings();

#endif
