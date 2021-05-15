#include "DShotRMT.h"

// ...fast forward
class DShotRMT;

DShotRMT::DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel) {
	dshot_config.gpio_num = gpio;
	dshot_config.pin_num = uint8_t(gpio);
	dshot_config.rmt_channel = rmtChannel;
	dshot_config.mem_block_num = uint8_t(RMT_CHANNEL_MAX - uint8_t(rmtChannel));

	// ...create clean packet
	encodeDShotRMT(DSHOT_NULL_PACKET);
}

DShotRMT::DShotRMT(uint8_t pin, uint8_t channel) {
	dshot_config.gpio_num = gpio_num_t(pin);
	dshot_config.pin_num = pin;
	dshot_config.rmt_channel = rmt_channel_t(channel);
	dshot_config.mem_block_num = (RMT_CHANNEL_MAX - channel);

	// ...create clean packet
	encodeDShotRMT(DSHOT_NULL_PACKET);
}

DShotRMT::~DShotRMT() {
	rmt_driver_uninstall(dshot_config.rmt_channel);
}

DShotRMT::DShotRMT(const DShotRMT& origin) {
	// ...write me	
}

void DShotRMT::init(dshot_mode_t dshot_mode) {
	dshot_config.mode = dshot_mode;
	dshot_config.clk_div = DSHOT_CLK_DIVIDER;

	switch (dshot_config.mode) {
		case DSHOT150:
			dshot_config.name_str = "DSHOT150";
			dshot_config.ticks_per_bit = 64; // ...Bit Period Time 6.67 탎
			dshot_config.ticks_zero_high = 25; // ...zero time 2.50 탎
			dshot_config.ticks_one_high = 50; // ...one time 5.00 탎
			break;
		case DSHOT300:
			dshot_config.name_str = "DSHOT300";
			dshot_config.ticks_per_bit = 32; // ...Bit Period Time 3.33 탎
			dshot_config.ticks_zero_high = 12; // ...zero time 1.25 탎
			dshot_config.ticks_one_high = 24; // ...one time 2.50 탎
			break;
		case DSHOT600:
			dshot_config.name_str = "DSHOT600";
			dshot_config.ticks_per_bit = 16; // ...Bit Period Time 1.67 탎
			dshot_config.ticks_zero_high = 6; // ...zero time 0.625 탎
			dshot_config.ticks_one_high = 12; // ...one time 1.25 탎
			break;
		case DSHOT1200:
			dshot_config.name_str = "DSHOT1200";
			dshot_config.ticks_per_bit = 8; // ...Bit Period Time 0.83 탎
			dshot_config.ticks_zero_high = 3; // ...zero time 0.313 탎
			dshot_config.ticks_one_high = 6; // ...one time 0.625 탎
			break;
		default:
			dshot_config.name_str = "DSHOT_OFF";
			dshot_config.ticks_per_bit = 0; // ...Bit Period Time endless
			dshot_config.ticks_zero_high = 0; // ...no bits, no time
			dshot_config.ticks_one_high = 0; // ......no bits, no time
			break;
	}

	dshot_config.ticks_zero_low = (dshot_config.ticks_per_bit - dshot_config.ticks_zero_high);
	dshot_config.ticks_one_low = (dshot_config.ticks_per_bit - dshot_config.ticks_one_high);

	config.rmt_mode = RMT_MODE_TX;
	config.channel = dshot_config.rmt_channel;
	config.gpio_num = dshot_config.gpio_num;
	config.mem_block_num = dshot_config.mem_block_num;
	config.clk_div = dshot_config.clk_div;

	config.tx_config.loop_en = false;
	config.tx_config.carrier_en = false;
	config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
	config.tx_config.idle_output_en = true;
	
	rmt_config(&config);
	rmt_driver_install(config.channel, 0, 0);
}

void DShotRMT::sendThrottle(uint16_t throttle_value, telemetric_request_t telemetric_request) {
	dshot_packet_t dshot_rmt_packet = { };

	if (throttle_value < DSHOT_THROTTLE_MIN) {
		throttle_value = DSHOT_THROTTLE_MIN;
	}

	if (throttle_value > DSHOT_THROTTLE_MAX) {
		throttle_value = DSHOT_THROTTLE_MAX;
	}
	
	dshot_rmt_packet.throttle_value = throttle_value;
	dshot_rmt_packet.telemetric_request = telemetric_request;
	dshot_rmt_packet.checksum = calculateChecksum(dshot_rmt_packet);

	writeDShotRMT(dshot_rmt_packet);
}

String DShotRMT::get_dshot_mode() {
	return dshot_config.name_str;
}

uint8_t DShotRMT::get_dshot_clock_div() {
	return dshot_config.clk_div;
}

rmt_item32_t* DShotRMT::encodeDShotRMT(uint16_t parsed_packet) {
	for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1) 	{
		if (parsed_packet & 0b1000000000000000) {
			// set one
			dshotItem[i].duration0 = dshot_config.ticks_one_high;
			dshotItem[i].level0 = 1;
			dshotItem[i].duration1 = dshot_config.ticks_one_low;
			dshotItem[i].level1 = 0;
		}
		else {
			// set zero
			dshotItem[i].duration0 = dshot_config.ticks_zero_high;
			dshotItem[i].level0 = 1;
			dshotItem[i].duration1 = dshot_config.ticks_zero_low;
			dshotItem[i].level1 = 0;
		}
	}

	// ...end marker added to each frame
	dshotItem[DSHOT_PAUSE_BIT].duration0 = DSHOT_PAUSE;
	dshotItem[DSHOT_PAUSE_BIT].level0 = 0;
	dshotItem[DSHOT_PAUSE_BIT].duration1 = 0;
	dshotItem[DSHOT_PAUSE_BIT].level1 = 0;

	return dshotItem;
}

uint16_t DShotRMT::calculateChecksum(const dshot_packet_t& dshot_packet_unsigned) {
	uint16_t packet = 0b0000000000000000;
	uint16_t chksum = 0b0000000000000000;

	// ...calculate checksum
	packet = (dshot_packet_unsigned.throttle_value << 1) | dshot_packet_unsigned.telemetric_request;;

	for (int i = 0; i < 3; i++) {
		chksum ^= packet;   // xor data by nibbles
		packet >>= 4;
	}

	chksum &= 0b0000000000001111;

	return chksum;
}

uint16_t DShotRMT::parseDShotPacket(const dshot_packet_t& signedDShotPacket) {
	uint16_t parsed_pack = NULL;

	parsed_pack = (signedDShotPacket.throttle_value << 1) | signedDShotPacket.telemetric_request;
	parsed_pack = (parsed_pack << 4) | signedDShotPacket.checksum;

	return parsed_pack;
}

dshot_packet_t DShotRMT::signDShotPacket(const uint16_t& throttle_value, const telemetric_request_t& telemetric_request) {
	dshot_packet_t signed_packet = { };

	signed_packet.isSigned = SIGNED;
	signed_packet.throttle_value = throttle_value;
	signed_packet.telemetric_request = telemetric_request;

	signed_packet.checksum = (calculateChecksum(signed_packet));

	return signed_packet;
}

void DShotRMT::writeDShotRMT(const dshot_packet_t& dshot_packet) {
	encodeDShotRMT(parseDShotPacket(dshot_packet));
	rmt_write_items(config.channel, dshotItem, DSHOT_PACKET_LENGTH, false);
}
