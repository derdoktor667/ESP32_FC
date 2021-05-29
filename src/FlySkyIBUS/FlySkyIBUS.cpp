
#include "FlySkyIBUS.h"

FlySkyIBUS* FlySkyIBUSFirst = nullptr;
FlySkyIBUS* FlySkyIBUSNext = nullptr;

// ...dirty helper workround
void IRAM_ATTR onTimer() {
	if (FlySkyIBUSFirst) {
		FlySkyIBUSFirst->process_ibus_data();
	}
}

FlySkyIBUS::FlySkyIBUS(HardwareSerial& ibus_serial, uint8_t ibus_timer_id, int8_t rx_pin, int8_t tx_pin) {
	ibus_serial.begin(IBUS_BAUD, SERIAL_8N1, rx_pin, tx_pin);
	this->ibus_config.ibus_input = &ibus_serial;
	this->ibus_config.timer_id = ibus_timer_id;
}

FlySkyIBUS::FlySkyIBUS(FlySkyIBUS const&) {

}

FlySkyIBUS& FlySkyIBUS::operator=(FlySkyIBUS const&) {
	// TODO: hier return-Anweisung eingeben
}

FlySkyIBUS::~FlySkyIBUS() {

}

void FlySkyIBUS::begin(uint32_t ibus_baud) {
	this->ibus_config.state = DISCARD;
	this->ibus_config.last_millis = millis();
	this->ibus_config.buffer_position = NULL;
	this->ibus_config.packet_length = NULL;
	this->ibus_config.chksum = NULL;
	this->ibus_config.lchksum = NULL;

	FlySkyIBUSNext = FlySkyIBUSFirst;

	// ...try to get in sync with RMT someday
	hw_timer_t* timer = nullptr;

	timer = timerBegin(ibus_config.timer_id, IBUS_TIMER_DIVIDER, true);
	timerAttachInterrupt(timer, &onTimer, true);
	timerAlarmWrite(timer, IBUS_TIMER_1_MS, true);
	timerAlarmEnable(timer);

	// ...first things first
	FlySkyIBUSFirst = this;
}

uint16_t FlySkyIBUS::read_Ibus_Channel(uint8_t channel_Nr) {
	if (channel_Nr < IBUS_MAX_CHANNELS) {
		return this->ibus_config.channel_data[channel_Nr];
	} else {
		return 0;
	}
}

uint16_t* FlySkyIBUS::read_All_Channels() {
	return this->ibus_config.channel_data;
}

void IRAM_ATTR FlySkyIBUS::process_ibus_data() {
	// ...call one after the other
	if (FlySkyIBUSNext) {
		FlySkyIBUSNext->process_ibus_data();
	}

	while (this->ibus_config.ibus_input->available() > 0) {
		// ...force ibus pause
		uint32_t now = millis();

		if (now - this->ibus_config.last_millis >= IBUS_PAUSE) {
			this->ibus_config.state = READ_IBUS_PACKET;
		}

		this->ibus_config.last_millis = now;

		// ...always read ibus stream
		auto ibus_raw = this->ibus_config.ibus_input->read();

		// ...state machine helps processing
		switch (ibus_config.state) {
			case READ_IBUS_PACKET:
				if (ibus_raw <= IBUS_MAX_LENGTH && ibus_raw > IBUS_OVERHEAD_LENGTH) {
					this->ibus_config.buffer_position = 0;
					this->ibus_config.packet_length = ibus_raw - IBUS_OVERHEAD_LENGTH;
					this->ibus_config.chksum = 0xFFFF - ibus_raw;

					// ...found a complete ibus packet
					this->ibus_config.state = PARSE_IBUS_PACKET;
				} else {
					// ...no packet found, nothing to do
					this->ibus_config.state = DISCARD;
				}
				break;

			case PARSE_IBUS_PACKET:
				this->ibus_config.buffer[this->ibus_config.buffer_position++] = ibus_raw;
				this->ibus_config.chksum -= ibus_raw;

				if (this->ibus_config.buffer_position == this->ibus_config.packet_length) {
					this->ibus_config.state = CHECK_PARSED_PACKET;
				}
				break;

			case CHECK_PARSED_PACKET:
				this->ibus_config.lchksum = ibus_raw;
				this->ibus_config.state = DECODE_IBUS_VALUES;
				break;

			case DECODE_IBUS_VALUES:
				// ...double check parsed packet
				if (this->ibus_config.chksum == (ibus_raw << 8) + this->ibus_config.lchksum) {
					
					// ...start has to be IBUS_COMMAND40
					if (this->ibus_config.buffer[0] == IBUS_COMMAND40) {

						// ...decode the packet
						for (uint8_t i = 1; i < IBUS_MAX_CHANNELS * 2 + 1; i += 2) {
							this->ibus_config.channel_data[i / 2] = this->ibus_config.buffer[i] | (this->ibus_config.buffer[i + 1] << 8);
						}
					}

					if (this->ibus_config.channel_data[THROTTLE] <= IBUS_VALUE_MIN) 	{
						this->ibus_config.channel_data[THROTTLE] = 0;
					}

					// ...all done
					this->ibus_config.state = DISCARD;
				}
				break;

			case DISCARD:
				break;

			default:
				break;
				
		}
	}
}
