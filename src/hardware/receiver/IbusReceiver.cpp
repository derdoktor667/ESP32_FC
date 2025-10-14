// IbusReceiver.cpp
//
// This file implements the IbusReceiver class, providing the concrete functionality
// for interacting with the Flysky i-BUS receiver protocol. It uses the FlyskyIBUS
// library to read channel data and determine failsafe status.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "src/hardware/receiver/IbusReceiver.h"

// Constructor for the IbusReceiver.
IbusReceiver::IbusReceiver(HardwareSerial &serialPort, gpio_num_t rxPin)
    : _ibus(serialPort, rxPin), _lastReceiveTime(0)
{
}

// Initializes the underlying FlyskyIBUS library.
void IbusReceiver::begin()
{
    _ibus.begin();
    _lastReceiveTime = millis(); // Initialize last receive time, assuming signal is present at startup
}

// The FlyskyIBUS library handles data reception in its internal loop or when getChannel() is called.
// This update method can be left empty or used for any periodic checks if needed.
void IbusReceiver::update()
{
    // All work is done by the underlying FlyskyIBUS library or on demand by getChannel().
    // No-op for this implementation.
}

// Gets the value of a specific iBUS channel.
uint16_t IbusReceiver::getChannel(int channel) const
{
    // The const_cast is necessary because the underlying library's getChannel
    // is not marked as const, but our interface requires it to be. This is safe
    // as the library's getChannel method only reads data and does not modify state.
    uint16_t value = const_cast<FlyskyIBUS &>(_ibus).getChannel(channel);
    // Update last receive time if a valid channel value is received.
    if (value > IBUS_INVALID_CHANNEL_VALUE) {
        const_cast<IbusReceiver*>(this)->_lastReceiveTime = millis();
    }
    return value;
}

// Checks the failsafe status from the iBUS receiver.
bool IbusReceiver::hasFailsafe() const
{
    // Failsafe is active if no signal has been received for a certain period.
    return (millis() - _lastReceiveTime > IBUS_SIGNAL_TIMEOUT_MS);
}
