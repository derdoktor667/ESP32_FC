// PpmReceiver.h
//
// This file defines the PpmReceiver class, a concrete implementation of the
// ReceiverInterface for the PPM (Pulse Position Modulation) protocol. It uses
// GPIO interrupts to decode the PPM signal and extract channel values.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef PPM_RECEIVER_H
#define PPM_RECEIVER_H

#include "src/hardware/receiver/ReceiverInterface.h"

class PpmReceiver : public ReceiverInterface
{
public:
    static constexpr int PPM_CHANNEL_COUNT = 8;            // Number of channels expected in the PPM stream
    static constexpr unsigned long PPM_SYNC_GAP_US = 4000; // Minimum time (in microseconds) for a pulse to be considered a sync pulse.

    // Constructor.
    // - ppmPin: The GPIO pin connected to the PPM signal.
    PpmReceiver(gpio_num_t ppmPin);

    void begin() override;
    void update() override;
    uint16_t getChannel(int channel) const override;
    bool hasFailsafe() const override;

private:
    gpio_num_t _ppmPin;                                         // The GPIO pin used for PPM input
    static constexpr unsigned long PPM_SIGNAL_TIMEOUT_MS = 500; // Milliseconds before signal is considered lost
    static constexpr uint16_t PPM_INVALID_CHANNEL_VALUE = 0;    // Value indicating an invalid channel
};

#endif // PPM_RECEIVER_H
