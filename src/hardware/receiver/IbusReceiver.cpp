#include "src/hardware/receiver/IbusReceiver.h"


// Constructor for the IbusReceiver.
IbusReceiver::IbusReceiver(HardwareSerial &serialPort, gpio_num_t rxPin)
    : _ibus(serialPort, rxPin), _lastReceiveTime(0) // Initialize _lastReceiveTime
{
}

// Initializes the underlying FlyskyIBUS library.
void IbusReceiver::begin()
{
    _ibus.begin();
    _lastReceiveTime = millis(); // Assume signal is present at startup
}

// Reads the latest data from the iBUS receiver.
// The FlyskyIBUS library handles this internally when getChannel is called,
// but we can call loop() here for good practice if the library supports it.
// The current FlyskyIBUS library reads in getChannel, so this can be empty
// or call a non-blocking read method if available. For now, we'll leave it empty
// as getChannel fetches the latest data.
void IbusReceiver::update()
{
    // The FlyskyIBUS library reads data on demand when getChannel() is called.
    // We will update _lastReceiveTime when getChannel() is called to indicate activity.
}

// Gets the value of a specific iBUS channel.
uint16_t IbusReceiver::getChannel(int channel) const
{
    // The const_cast is necessary because the underlying library's getChannel
    // is not marked as const, but our interface requires it to be.
    // This is safe as getChannel in the library only reads data.
    uint16_t value = const_cast<FlyskyIBUS &>(_ibus).getChannel(channel);
    // Update last receive time if a valid channel value is received
    if (value > 0) { // Assuming 0 is an invalid/unreceived value
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
