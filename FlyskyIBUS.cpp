#include <Arduino.h>
#include "FlyskyIBUS.h"

FlyskyIBUS IBus;

void FlyskyIBUS::begin(HardwareSerial &serial) {
    serial.begin(115200);
    begin((Stream&)serial);
}

void FlyskyIBUS::begin(Stream& stream) {
    this->stream = &stream;
    this->state = DISCARD;
    this->last = millis();
    this->ptr = 0;
    this->len = 0;
    this->chksum = 0;
    this->lchksum = 0;
}

void FlyskyIBUS::loop(void) {
    while (stream->available() > 0)   {
        uint32_t now = millis();
        if (now - last >= PROTOCOL_TIMEGAP)     {
            state = GET_LENGTH;
        }
        last = now;

        uint8_t v = stream->read();

        switch (state)     {
            case GET_LENGTH:
                if (v <= IBUS_LENGTH)         {
                    ptr = 0;
                    len = v - PROTOCOL_OVERHEAD;
                    chksum = 0xFFFF - v;
                    state = GET_DATA;
                }
                else         {
                    state = DISCARD;
                }
                break;

            case GET_DATA:
                buffer[ptr++] = v;
                chksum -= v;
                if (ptr == len)         {
                    state = GET_CHKSUML;
                }
                break;

            case GET_CHKSUML:
                lchksum = v;
                state = GET_CHKSUMH;
                break;

            case GET_CHKSUMH:
                // Validate checksum
                if (chksum == (v << 8) + lchksum)         {
                    // Execute command - we only know command 0x40
                    switch (buffer[0])           {
                        case PROTOCOL_COMMAND40:
                            // Valid - extract channel data
                            for (uint8_t i = 1; i < IBUS_CHANNELS * 2 + 1; i += 2)               {
                                channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
                            }
                            break;

                        default:
                            break;
                    }
                }
                state = DISCARD;
                break;

            case DISCARD:
            default:
                break;
        }
    }
}

uint16_t FlyskyIBUS::readChannel(uint8_t channelNr) {
    if (channelNr < IBUS_CHANNELS)   {
        return channel[channelNr];
    }
    else   {
        return 0;
    }
}
