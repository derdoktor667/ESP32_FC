#include "IbusReceiver.h"
#include "flight_controller.h" // For settings access

// Constructor for the IbusReceiver.
IbusReceiver::IbusReceiver(HardwareSerial &serialPort, gpio_num_t rxPin)
    : _ibus(serialPort, rxPin)
{
}

// Initializes the underlying FlyskyIBUS library.
void IbusReceiver::begin()
{
    _ibus.begin();
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
    // No explicit update call is needed in the main loop for this specific library.
}

// Gets the value of a specific iBUS channel.
uint16_t IbusReceiver::getChannel(int channel) const
{
    // The const_cast is necessary because the underlying library's getChannel
    // is not marked as const, but our interface requires it to be.
    // This is safe as getChannel in the library only reads data.
    return const_cast<FlyskyIBUS &>(_ibus).getChannel(channel);
}

// Checks the failsafe status from the iBUS receiver.
bool IbusReceiver::hasFailsafe() const
{
    uint16_t failsafe_channel_value = getChannel(CHANNEL_FAILSAFE);
    return failsafe_channel_value > settings.receiver.failsafeThreshold;
}
