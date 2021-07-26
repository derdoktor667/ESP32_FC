## ESP32_FC - RMT Edition

 ...some experiments with the ESP32 RMT generating DSHOT pulses for BLDC ESCs running BLHeli_S

### The DShot Protocol
The DSHOT protocol consists of transmitting 16-bit packets to the ESCs: 11-bit throttle value,  1-bit to request telemetry and a 4-bit checksum. There are three major protocol speeds: DSHOT150, DSHOT300 and DSHOT600.

#### DShot -Timing:

  DSHOT  | Bitrate  |  TH1   |   TH0  | Bit Time µs | Frame Time µs
--------:|---------:|-------:|-------:|------------:|-------------:
**150**  | 150kbit/s|  5.00  |  2.50  |     6.67    |    106.72
**300**  | 300kbit/s|  2.50  |  1.25  |     3.33    |     53.28
**600**  | 600kbit/s|  1.25  |  0.625 |     1.67    |     26.72
**1200** |1200kbit/s|  0.625 |  0.313 |     0.83    |     13.28

#### Calculating the CRC
The checksum is calculated over the throttle value and the telemetry bit, so the “first” 12 bits our value in the following example:

    crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;

### Bidirectional DSHOT
Biderictional DSHOT is also known as inverted DSHOT, because the signal level is inverted, so 1 is low and a 0 is high. This is done in order to let the ESC know, that we are operating in bidirectional mode and that it should be sending back eRPM telemetry packages.

#### Calculating the Bidirectional CRC
The calculation of the checksum is basically the same, just before the last step the values are inverted:

    crc = (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;

----

*References:* 
- [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/) 

- [DSHOT in the Dark](https://dmrlawson.co.uk/index.php/2017/12/04/dshot-in-the-dark/)
