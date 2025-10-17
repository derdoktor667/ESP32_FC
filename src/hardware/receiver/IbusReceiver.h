// IbusReceiver.h
//
// This file defines the IbusReceiver class, a concrete implementation of the
// ReceiverInterface for the Flysky i-BUS protocol. It acts as an adapter for
// the FlyskyIBUS library, making it compatible with the flight controller's
// generic receiver interface.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef IBUS_RECEIVER_H
#define IBUS_RECEIVER_H

#include "src/hardware/receiver/ReceiverInterface.h"
#include <FlyskyIBUS.h>

// iBUS protocol implementation of the ReceiverInterface.
// This class acts as a wrapper or adapter for the FlyskyIBUS library,
// making it compatible with the flight controller's generic receiver interface.
class IbusReceiver : public ReceiverInterface
{
public:
    // Constructor.
    // - serialPort: The hardware serial port the iBUS receiver is connected to.
    // - rxPin: The GPIO pin number for the serial RX line.
    IbusReceiver(HardwareSerial &serialPort, gpio_num_t rxPin);

    void begin() override;
    void update() override;
    uint16_t getChannel(int channel) const override;
    bool hasFailsafe() const override;

private:
    mutable FlyskyIBUS _ibus; // The underlying iBUS library object
    static constexpr unsigned long IBUS_SIGNAL_TIMEOUT_MS = 500; // Milliseconds before signal is considered lost
    static constexpr uint16_t IBUS_INVALID_CHANNEL_VALUE = 0; // Value indicating an invalid or unreceived channel
};

#endif // IBUS_RECEIVER_H
