//
// Name:		FlySkyIBUS.h
// Created: 	20.03.2021 00:49:15
// Author:  	derdoktor667
//

#pragma once

#include <Arduino.h>

//
// max 14 channels in this lib (with messagelength of 0x20 there is room for 14 channels)
//
// Example set of bytes coming over the iBUS line for setting servos:
// 20 40 DB 5 DC 5 54 5 DC 5 E8 3 D0 7 D2 5 E8 3 DC 5 DC 5 DC 5 DC 5 DC 5 DC 5 DA F3
//
//  ...decoded:
// Packet Start: 20
// Packet Command: 40
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
// 
// Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
//

constexpr auto IBUS_MAX_LENGTH = 32;
constexpr auto IBUS_OVERHEAD_LENGTH = 3;
constexpr auto IBUS_PAUSE = 3;
constexpr auto IBUS_MAX_CHANNELS = 14;
constexpr auto IBUS_SENSORS_MAX = 10;
constexpr auto IBUS_BAUD = 115200;
constexpr auto IBUS_TIMER_DIVIDER = 80;
constexpr auto IBUS_TIMER_SCALE = F_CPU / 1000000L;
constexpr auto IBUS_TIMER_1_MS = 1000;

constexpr auto IBUS_PACKET_START = 0x20;
constexpr auto IBUS_COMMAND40 = 0x40;

constexpr auto IBUS_VALUE_MIN = 988;
constexpr auto IBUS_VALUE_MAX = 2012;
constexpr auto IBUS_SAFETY_VALUE = 7;
constexpr auto IBUS_RANGE = IBUS_VALUE_MAX - IBUS_VALUE_MIN;

// ...only HardwareSerial aka UART supported at the moment
class FlySkyIBUS {
	public:
	FlySkyIBUS(HardwareSerial& ibus_serial = Serial2, uint8_t ibus_timer_id = 0, int8_t rx_pin = -1, int8_t tx_pin = -1);
	FlySkyIBUS(FlySkyIBUS const&);
	~FlySkyIBUS();

	void begin(uint32_t ibus_baud = IBUS_BAUD);

	// ...returns a single channel value
	uint16_t get_IBUS_Channel(uint8_t channel_Nr);

	// ...returns all decoded channels as an array
	uint16_t* get_IBUS_Channels();

	// ...move to private somehow
	void process_ibus_data();

	private:
    typedef enum ibus_state_e {
		READ_IBUS_PACKET,
		PARSE_IBUS_PACKET,
		CHECK_PARSED_PACKET,
		DECODE_IBUS_VALUES,
		DISCARD
	} ibus_state_t;

	struct ibus_config_s {
		ibus_state_t state = DISCARD;
		HardwareSerial* ibus_input = nullptr;
		uint8_t buffer[IBUS_MAX_LENGTH] = { };
		uint8_t timer_id = 0;
		uint8_t buffer_position = 0;
		uint8_t packet_length = 0;
		uint16_t channel_data[IBUS_MAX_CHANNELS] = { };
		uint16_t chksum = 0;
		uint8_t lchksum = 0;
		uint8_t sensor_count = 0;
		uint32_t last_millis = 0;
		uint32_t fs_counter = 0;
		uint32_t fs_last_counter = 0;
	} ibus_config;
};
