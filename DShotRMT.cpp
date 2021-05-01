#include "DShotRMT.h"

DShotRMT::DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel, bool wait) {
	// ...safety first
	dshot_config = {};
	
	dshot_config.gpio_num = gpio;
	dshot_config.pin_num = gpio;
	dshot_config.rmt_channel = rmtChannel;
	dshot_config.mem_block_num = uint8_t(RMT_CHANNEL_MAX - (int)rmtChannel);

	// initialize cmd buffer
	setData(0);

	// DShot packet delay + RMT end marker
	dshot_command[16].duration0 = DSHOT_PAUSE;
	dshot_command[16].level0 = 0;
	dshot_command[16].duration1 = 0;
	dshot_command[16].level1 = 0;
}

DShotRMT::~DShotRMT() {
	// TODO write destructor
}

void DShotRMT::init(dshot_mode_t mode) {
	dshot_config.mode = mode;
	dshot_config.clk_div = DSHOT_CLK_DIVIDER;

	switch (dshot_config.mode) {
		case DSHOT150:
			dshot_config.name_str = "DSHOT150";
			dshot_config.ticks_per_packet = 67; // ...Bit Period Time 6.67 탎
			dshot_config.ticks_zero_high = 25; // ...zero time 2.50 탎
			dshot_config.ticks_one_high = 50; // ...one time 5.00 탎
			break;
		case DSHOT300:
			dshot_config.name_str = "DSHOT300";
			dshot_config.ticks_per_packet = 33; // ...Bit Period Time 3.33 탎
			dshot_config.ticks_zero_high = 12; // ...zero time 1.25 탎
			dshot_config.ticks_one_high = 25; // ...one time 2.50 탎
			break;
		case DSHOT600:
			dshot_config.name_str = "DSHOT600";
			dshot_config.ticks_per_packet = 17; // ...Bit Period Time 1.67 탎
			dshot_config.ticks_zero_high = 6; // ...zero time 0.625 탎
			dshot_config.ticks_one_high = 13; // ...one time 1.25 탎
			break;
		case DSHOT1200:
			dshot_config.name_str = "DSHOT1200";
			dshot_config.ticks_per_packet = 8; // ...Bit Period Time 0.83 탎
			dshot_config.ticks_zero_high = 3; // ...zero time 0.313 탎
			dshot_config.ticks_one_high = 6; // ...one time 0.625 탎
			break;
		default:
			break;
	}

	dshot_config.ticks_zero_low = (dshot_config.ticks_per_packet - dshot_config.ticks_zero_high);
	dshot_config.ticks_one_low = (dshot_config.ticks_per_packet - dshot_config.ticks_one_high);

	config.rmt_mode = RMT_MODE_TX;
	config.channel = dshot_config.rmt_channel;
	config.gpio_num = dshot_config.gpio_num;
	config.mem_block_num = dshot_config.mem_block_num;
	config.clk_div = dshot_config.clk_div;

	config.tx_config.loop_en = false;
	config.tx_config.carrier_en = false;
	config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
	config.tx_config.idle_output_en = false;

	rmt_config(&config);
	rmt_driver_install(config.channel, 0, 0);
	
	for (int i = 0; i < 20; i++) {
		writeData(0, false);
	}

	dshot_packet_t repeat_packet = { DSHOT_THROTTLE_MIN, 0 };

	repeatPacketTicks(repeat_packet, DSHOT_ARM_DELAY);
}

void DShotRMT::sendThrottle(uint16_t throttle) {
	if (throttle < DSHOT_THROTTLE_MIN) {
		throttle = DSHOT_THROTTLE_MIN;
	}
	
	if (throttle > DSHOT_THROTTLE_MAX) {
		throttle = DSHOT_THROTTLE_MAX;
	}

	dshot_packet_t throttle_packet = { throttle, 0 };

	writePacket(throttle_packet, false);
}

void DShotRMT::setReversed(bool reversed) {
	dshot_packet_t reverse_packet = { DSHOT_THROTTLE_MIN, 0 };

	repeatPacketTicks(reverse_packet, 200 / portTICK_PERIOD_MS);
	repeatPacket( { reversed ? DIGITAL_CMD_SPIN_DIRECTION_REVERSED : DIGITAL_CMD_SPIN_DIRECTION_NORMAL, 1 }, 10);
}

void DShotRMT::beep() {
	writePacket({ DIGITAL_CMD_BEEP1, 0 }, true);
	vTaskDelay(260 / portTICK_PERIOD_MS);
}

String DShotRMT::get_dshot_mode() {
	return dshot_config.name_str;
}

uint8_t DShotRMT::get_dshot_clock_div() {
	return dshot_config.clk_div;
}

void DShotRMT::setData(uint16_t data) {
	for (int i = 0; i < 16; i++, data <<= 1) 	{
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
}

volatile uint8_t DShotRMT::checksum(uint16_t data) {
	uint16_t csum = 0;

	for (int i = 0; i < 3; i++) 	{
		csum ^= data;
		data >>= 4;
	}

	return csum &= 0xf;
}

void DShotRMT::writeData(uint16_t data, bool wait) {
	setData(data);
	rmt_write_items(config.channel, dshot_command, DSHOT_PACKET_LENGTH, wait);
}

void DShotRMT::writePacket(dshot_packet_t packet, bool wait) {
	uint16_t data = packet.payload;

	data <<= 1;
	data |= packet.telemetry;

	data = (data << 4) | checksum(data);

	writeData(data, wait);
}

void DShotRMT::repeatPacket(dshot_packet_t packet, int n) {
	for (int i = 0; i < n; i++) 	{
		writePacket(packet, false);
		portYIELD();
	}
}

void DShotRMT::repeatPacketTicks(dshot_packet_t packet, TickType_t ticks) {
	TickType_t repeatStop = xTaskGetTickCount() + ticks;

	while (xTaskGetTickCount() < repeatStop) 	{
		writePacket(packet, false);
		vTaskDelay(1);
	}
}
