#ifndef PPM_RECEIVER_H
#define PPM_RECEIVER_H

#include "ReceiverInterface.h"

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
    gpio_num_t _ppmPin; // The GPIO pin used for PPM input
    unsigned long _lastReceiveTime; // Timestamp of the last successfully received PPM frame
    static constexpr unsigned long PPM_SIGNAL_TIMEOUT_MS = 500; // Milliseconds before signal is considered lost
};

#endif // PPM_RECEIVER_H
