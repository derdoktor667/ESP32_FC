// ReceiverInterface.h
//
// This file defines the abstract base class for RC receiver protocols.
// It establishes a common interface that all concrete receiver implementations
// must adhere to, ensuring modularity and interchangeability of different
// receiver hardware and protocols (e.g., iBUS, PPM, SBUS).
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef RECEIVER_INTERFACE_H
#define RECEIVER_INTERFACE_H

#include <Arduino.h>

// An abstract interface for different RC receiver protocols.
// This class defines a common set of functions that any receiver implementation
// (like iBUS, SBUS, PPM) must provide. The flight controller will interact
// with this interface, making the underlying protocol interchangeable.
class ReceiverInterface
{
public:
    // Virtual destructor.
    virtual ~ReceiverInterface() {}

    // Initializes the receiver hardware and protocol.
    virtual void begin() = 0;

    // Reads the latest data from the receiver.
    // This should be called once per flight loop.
    virtual void update() = 0;

    // Gets the normalized value of a specific channel.
    // The channel index is passed as an argument (e.g., CHANNEL_ROLL).
    // Returns the value of the channel, typically in the range 1000-2000.
    virtual uint16_t getChannel(int channel) const = 0;

    // Checks if the receiver is currently in a failsafe state.
    // Returns true if failsafe is active, false otherwise.
    virtual bool hasFailsafe() const = 0;
};

#endif // RECEIVER_INTERFACE_H