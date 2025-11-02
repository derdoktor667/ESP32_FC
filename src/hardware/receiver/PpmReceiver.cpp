// PpmReceiver.cpp
//
// This file implements the PpmReceiver class, providing the concrete functionality
// for decoding PPM (Pulse Position Modulation) signals using GPIO interrupts.
// It measures pulse widths to determine channel values and detects failsafe conditions.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "src/hardware/receiver/PpmReceiver.h"
#include "src/config/config.h"

// --- Static variables for the Interrupt Service Routine (ISR) ---

// Volatile is crucial here, as these variables are modified by an ISR and read in the main loop.
static volatile uint16_t _ppm_channel_values[PPM_CHANNEL_COUNT] = {0};
static volatile int _ppm_current_channel = 0;
static volatile unsigned long _ppm_last_valid_pulse_time = 0;   // Stores the microsecond timestamp of the last valid pulse edge.
static volatile unsigned long _ppm_last_valid_frame_millis = 0; // Stores the millisecond timestamp of the last valid PPM frame sync.

// Interrupt Service Routine to handle incoming PPM pulses.
// IRAM_ATTR ensures this code is placed in IRAM for faster execution, which is critical for ISRs.
void IRAM_ATTR _handle_ppm_interrupt()
{
    unsigned long now = micros();
    unsigned long pulse_width = now - _ppm_last_valid_pulse_time;
    _ppm_last_valid_pulse_time = now; // Update on every rising edge

    if (pulse_width > PPM_SYNC_GAP_US)
    {
        // A long gap indicates the start of a new frame (sync pulse).
        _ppm_current_channel = 0;
        _ppm_last_valid_frame_millis = millis(); // Update last valid frame time
    }
    else
    {
        // A short gap is a channel pulse.
        if (_ppm_current_channel < PPM_CHANNEL_COUNT)
        {
            // PPM values are typically between 1000 and 2000 microseconds.
            _ppm_channel_values[_ppm_current_channel] = pulse_width;
            _ppm_current_channel++;
        }
    }
}

// --- PpmReceiver Class Implementation ---

PpmReceiver::PpmReceiver(gpio_num_t ppmPin)
    : _ppmPin(ppmPin)
{
}

// Initializes the PPM receiver by setting up the GPIO pin and attaching the interrupt.
void PpmReceiver::begin()
{
    pinMode(_ppmPin, INPUT_PULLUP); // Use a pull-up to ensure a stable line if the receiver is disconnected
    attachInterrupt(digitalPinToInterrupt(_ppmPin), _handle_ppm_interrupt, RISING);
    _ppm_last_valid_pulse_time = micros();   // Initialize with current time, assuming signal is present at startup
    _ppm_last_valid_frame_millis = millis(); // Initialize for failsafe detection
}

// The PPM signal is decoded by interrupts in the background.
// This function does not need to perform any active work.
void PpmReceiver::update()
{
    // All work is done in the ISR. No-op.
}

// Gets the value of a specific PPM channel.
// The channel index is passed as an argument.
// Returns the channel value (typically 1000-2000 microseconds).
uint16_t PpmReceiver::getChannel(int channel) const
{
    if (channel < 0 || channel >= PPM_CHANNEL_COUNT)
    {
        return PPM_INVALID_CHANNEL_VALUE; // Return 0 for invalid channels
    }
    // Read the volatile variable into a local variable to ensure atomicity.
    return _ppm_channel_values[channel];
}

// Checks for failsafe condition.
bool PpmReceiver::hasFailsafe() const
{
    // Failsafe is active if no signal has been received for a certain period.
    return (millis() - _ppm_last_valid_frame_millis > PPM_SIGNAL_TIMEOUT_MS);
}
