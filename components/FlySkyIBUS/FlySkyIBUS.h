
#ifndef  FlySkyIBUS_h
#define FlySkyIBUS_h

//
// supports max 14 channels in this lib (with messagelength of 0x20 there is room for 14 channels)
//
// Example set of bytes coming over the iBUS line for setting servos:
// 20 40 DB 5 DC 5 54 5 DC 5 E8 3 D0 7 D2 5 E8 3 DC 5 DC 5 DC 5 DC 5 DC 5 DC 5 DA F3
//
//  ...decoded:
// ibus packet start: 20
// ibus command code: 40
// 
// Channel 0: DB 5  -> value 0x5DB
// Channel 1: DC 5  -> value 0x5DC
// Channel 2: 54 5  -> value 0x554
// Channel 3: DC 5  -> value 0x5DC
// Channel 4: E8 3  -> value 0x3E8
// Channel 5: D0 7  -> value 0x7D0
// Channel 6: D2 5  -> value 0x5D2
// Channel 7: E8 3  -> value 0x3E8
// Channel 8: DC 5  -> value 0x5DC
// Channel 9: DC 5  -> value 0x5DC
// Channel 10: DC 5 -> value 0x5DC
// Channel 11: DC 5 -> value 0x5DC
// Channel 12: DC 5 -> value 0x5DC
// Channel 13: DC 5 -> value 0x5DC
// Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
//

#include <Arduino.h>
#include <HardwareSerial.h>

constexpr auto IBUS_LENGTH = 0x20;
constexpr auto IBUS_OVERHEAD = 3;
constexpr auto IBUS_PAUSE = 3;
constexpr auto IBUS_MAX_CHANNELS = 14;
constexpr auto IBUS_SENSORS_MAX = 10;
constexpr auto IBUS_BAUD = 115200;
constexpr auto IBUS_TIMER_DIVIDER = 80;

constexpr auto IBUS_COMMAND40 = 0x40;
constexpr auto IBUS_COMMAND_DISCOVER = 0x80;
constexpr auto IBUS_COMMAND_TYPE = 0x90;
constexpr auto IBUS_COMMAND_VALUE = 0xA0;

// ...only HardwareSerial aka UART supported at the moment
class FlySkyIBUS {
	public:
	FlySkyIBUS(HardwareSerial& ibus_serial, uint8_t ibus_timer_id = 0, int8_t rx_pin = -1, int8_t tx_pin = -1);
	~FlySkyIBUS();

	void begin(uint32_t ibus_baud = IBUS_BAUD);

	uint16_t readChannel(uint8_t channelNr);
	uint8_t addSensor(uint8_t type, uint8_t len = 2);
	void setSensorMeasurement(uint8_t adr, int32_t value);

	// ...move to private
	void process_ibus_data();

	volatile uint8_t cnt_poll; // count received number of sensor poll messages
	volatile uint8_t cnt_sensor; // count times a sensor value has been sent back
	volatile uint8_t cnt_rec; // count received number of servo messages

	typedef struct sensor_info {
		uint8_t sensorType;             // sensor type (0,1,2,3, etc)
		uint8_t sensorLength;           // data length for defined sensor (can be 2 or 4)
		int32_t sensorValue;            // sensor data for defined sensors (16 or 32 bits)
	} sensor_info_t;

	sensor_info sensors[IBUS_SENSORS_MAX];

	private:
	struct ibus_config_s {
		uint8_t state;
		HardwareSerial* ibus_input;
		uint8_t buffer[IBUS_LENGTH];
		uint8_t timer_id;
		uint8_t ptr_buffer;
		uint8_t packet_length;
		uint16_t channel_data[IBUS_MAX_CHANNELS];
		uint16_t chksum;
		uint8_t lchksum;
		uint8_t sensor_count = 0;
		uint32_t last_millis;
	} ibus_config;

	enum ibus_state_e {
		GET_LENGTH,
		GET_DATA,
		GET_CHKSUML,
		GET_CHKSUMH,
		DISCARD
	};
};

#endif // ! FlySkyIBUS_h
