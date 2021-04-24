#include "DShotRMT.h"

DShotRMT::DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel, bool wait) {
	dshot_config.dshot_gpio = gpio;
	dshot_config.dshot_pin = gpio;
	dshot_config.dshot_rmtChannel = rmtChannel;
	dshot_config.dshot_mem_block_num = (RMT_CHANNEL_MAX - (int)rmtChannel);

	// initialize cmd buffer
	setData(0);

	// DShot packet delay + RMT end marker
	_dshotCmd[16].duration0 = DSHOT_PAUSE;
	_dshotCmd[16].level0 = 0;
	_dshotCmd[16].duration1 = 0;
	_dshotCmd[16].level1 = 0;
}

DShotRMT::~DShotRMT() {
	// TODO write destructor
}

void DShotRMT::init(dshot_mode_t mode) {
	dshot_config.dshot_mode = mode;
	dshot_config.dshot_clk_div = 8;

	switch (dshot_config.dshot_mode) {
		case DSHOT150:
			dshot_config.dshot_name = "DSHOT150";
			dshot_config.dshot_packet_ticks = 67;
			dshot_config.dshot_t0h = 25;
			dshot_config.dshot_t1h = 50;
			break;
		case DSHOT300:
			dshot_config.dshot_name = "DSHOT300";
			dshot_config.dshot_packet_ticks = 33;
			dshot_config.dshot_t0h = 12;
			dshot_config.dshot_t1h = 25;
			break;
		case DSHOT600:
			dshot_config.dshot_name = "DSHOT600";
			dshot_config.dshot_packet_ticks = 17;
			dshot_config.dshot_t0h = 6;
			dshot_config.dshot_t1h = 13;
			break;
		case DSHOT1200:
			dshot_config.dshot_name = "DSHOT1200";
			dshot_config.dshot_packet_ticks = 8;
			dshot_config.dshot_t0h = 3;
			dshot_config.dshot_t1h = 6;
			break;
		default:
			break;
	}

	dshot_config.dshot_t0l = (dshot_config.dshot_packet_ticks - dshot_config.dshot_t0h);
	dshot_config.dshot_t1l = (dshot_config.dshot_packet_ticks - dshot_config.dshot_t1h);

	config.channel = dshot_config.dshot_rmtChannel;
	config.rmt_mode = RMT_MODE_TX;
	config.gpio_num = dshot_config.dshot_gpio;
	config.mem_block_num = dshot_config.dshot_mem_block_num;
	config.clk_div = dshot_config.dshot_clk_div;

	config.tx_config.loop_en = false;
	config.tx_config.carrier_en = false;
	config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
	config.tx_config.idle_output_en = true;

	rmt_config(&config);
	rmt_driver_install(config.channel, 0, 0);
	
	for (int i = 0; i < DSHOT_PACKET_LENGTH; i++) {
		writeData(0, true);
	}

	dshot_packet_t repeat_packet = { DSHOT_THROTTLE_MIN, 0 };

	repeatPacketTicks(repeat_packet, DSHOT_ARM_DELAY);
}

void DShotRMT::sendThrottle(uint16_t throttle) {
	if (throttle < DSHOT_THROTTLE_MIN || throttle > DSHOT_THROTTLE_MAX) {
		
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
	return dshot_config.dshot_name;
}

void DShotRMT::setData(uint16_t data) {
	for (int i = 0; i < 16; i++, data <<= 1) 	{
		if (data & 0x8000) {
			// set one
			_dshotCmd[i].duration0 = dshot_config.dshot_t1h;
			_dshotCmd[i].level0 = 1;
			_dshotCmd[i].duration1 = dshot_config.dshot_t1l;
			_dshotCmd[i].level1 = 0;
		}
		else {
			// set zero
			_dshotCmd[i].duration0 = dshot_config.dshot_t0h;
			_dshotCmd[i].level0 = 1;
			_dshotCmd[i].duration1 = dshot_config.dshot_t0l;
			_dshotCmd[i].level1 = 0;
		}
	}
}

volatile uint8_t DShotRMT::checksum(uint16_t data) {
	uint16_t csum = 0;

	for (int i = 0; i < 3; i++) 	{
		csum ^= data;
		data >>= 4;
	}

	return csum & 0xf;
}

void DShotRMT::writeData(uint16_t data, bool wait) {
	setData(data);
	rmt_write_items(config.channel, _dshotCmd, DSHOT_PACKET_LENGTH, wait);
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
		writePacket(packet, true);
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
