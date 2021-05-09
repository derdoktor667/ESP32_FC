#include "DShotRMT.h"

// ...fast forward
class DShotRMT;

DShotRMT::DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel) {
	dshot_config.gpio_num = gpio;
	dshot_config.pin_num = gpio;
	dshot_config.rmt_channel = rmtChannel;
	dshot_config.mem_block_num = uint8_t(RMT_CHANNEL_MAX - (int)rmtChannel);

	// ...create clean packet
	encodeDShotRMT(0);
}

DShotRMT::~DShotRMT() {
	rmt_driver_uninstall(dshot_config.rmt_channel);
}

void DShotRMT::init(dshot_mode_t dshot_mode, telemetric_request_t telemetric_request) {
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

void DShotRMT::sendThrottle(uint16_t throttle_value) {
	if (throttle_value < DSHOT_THROTTLE_MIN) {
		throttle_value = DSHOT_THROTTLE_MIN;
	}

	if (throttle_value > DSHOT_THROTTLE_MAX) {
		throttle_value = DSHOT_THROTTLE_MAX;
	}
		
	dshot_packet_t throttle_packet = signDShotPacket(throttle_value);

	writeDShotRMT(throttle_packet);
}

void DShotRMT::setReversed(bool isReversed) {
	dshot_packet_t reverse_packet = { DSHOT_THROTTLE_MIN, 0 };

	repeatPacketTicks(reverse_packet, 200 / portTICK_PERIOD_MS);
	repeatPacket( { isReversed ? DIGITAL_CMD_SPIN_DIRECTION_REVERSED : DIGITAL_CMD_SPIN_DIRECTION_NORMAL, 1 }, 10);
}

void DShotRMT::beep() {
	dshot_packet_t beep_packet = signDShotPacket(DIGITAL_CMD_BEEP1);

	writeDShotRMT(beep_packet);
}

DShotRMT& DShotRMT::set_dshot_mode(dshot_mode_t dshot_mode) {
	dshot_config.mode = dshot_mode;
	return *this;
}

String DShotRMT::get_dshot_mode() {
	return dshot_config.name_str;
}

uint8_t DShotRMT::get_dshot_clock_div() {
	return dshot_config.clk_div;
}

rmt_item32_t DShotRMT::encodeDShotRMT(uint16_t data) {
	for (int i = 0; i < DSHOT_PAUSE_BIT; i++, data <<= 1) 	{
		if (data & 0x8000) {
			// set one
			dshot_command[i].duration0 = dshot_config.ticks_one_high;
			dshot_command[i].level0 = 1;
			dshot_command[i].duration1 = dshot_config.ticks_one_low;
			dshot_command[i].level1 = 0;
		}
		else {
			// set zero
			dshot_command[i].duration0 = dshot_config.ticks_zero_high;
			dshot_command[i].level0 = 1;
			dshot_command[i].duration1 = dshot_config.ticks_zero_low;
			dshot_command[i].level1 = 0;
		}
	}

	// ...end marker
	dshot_command[DSHOT_PAUSE_BIT].duration0 = DSHOT_PAUSE;
	dshot_command[DSHOT_PAUSE_BIT].level0 = 0;
	dshot_command[DSHOT_PAUSE_BIT].duration1 = 0;
	dshot_command[DSHOT_PAUSE_BIT].level1 = 0;

	return *dshot_command;
}

dshot_packet_t DShotRMT::signDShotPacket(uint16_t throttle_value, bool telemetric_request) {
	dshot_packet_t temp_pack = {};

	temp_pack.throttle_value = throttle_value;
	temp_pack.telemetric_request = telemetric_request;

	uint16_t packet = (throttle_value << 1) | telemetric_request;

	// ...calculate checksum
	uint16_t chksum = 0;
	uint16_t csum_data = packet;

	for (int i = 0; i < 3; i++) {
		chksum ^= csum_data;   // xor data by nibbles
		csum_data >>= 4;
	}
	chksum &= 0xf;

	// ...append checksum
	temp_pack.checksum = chksum;

	return temp_pack;
}

void DShotRMT::writeDShotRMT(dshot_packet_t packet) {
	uint16_t packet_parsed = (packet.throttle_value << 1) | packet.telemetric_request;
	packet_parsed = (packet_parsed << 4) | packet.checksum;

	encodeDShotRMT(packet_parsed);
	rmt_write_items(config.channel, dshot_command, DSHOT_PACKET_LENGTH, false);
}

void DShotRMT::repeatPacket(dshot_packet_t packet, int n) {
	for (int i = 0; i < n; i++) 	{
		writeDShotRMT(packet);
		portYIELD();
	}
}

void DShotRMT::repeatPacketTicks(dshot_packet_t packet, TickType_t ticks) {
	TickType_t repeatStop = xTaskGetTickCount() + ticks;

	while (xTaskGetTickCount() < repeatStop) 	{
		writeDShotRMT(packet);
		vTaskDelay(1);
	}
}
