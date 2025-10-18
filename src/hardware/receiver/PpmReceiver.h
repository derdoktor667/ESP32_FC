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

// The number of channels expected in the PPM stream
constexpr int PPM_CHANNEL_COUNT = 8;

// PPM protocol implementation of the ReceiverInterface.
//
// This class uses a GPIO interrupt to decode a standard PPM signal.
// It measures the time between rising edges of the PPM pulses to determine
// the value for each channel.
class PpmReceiver : public ReceiverInterface
{
public:
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
