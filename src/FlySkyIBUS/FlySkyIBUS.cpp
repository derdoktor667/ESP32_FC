
#include "FlySkyIBUS.h"

extern const long F_CPU_RMT;

FlySkyIBUS* FlySkyIBUSFirst = nullptr;
FlySkyIBUS* FlySkyIBUSNext = nullptr;

// ...dirty helper workround
void onTimer() {
	if (FlySkyIBUSFirst) {
		FlySkyIBUSFirst->process_ibus_data();
	}
}

FlySkyIBUS::FlySkyIBUS(HardwareSerial& ibus_serial, uint8_t ibus_timer_id, int8_t rx_pin, int8_t tx_pin) {
	ibus_serial.begin(IBUS_BAUD, SERIAL_8N1, rx_pin, tx_pin);
	ibus_config.ibus_input = &ibus_serial;
	ibus_config.timer_id = ibus_timer_id;
}

FlySkyIBUS::~FlySkyIBUS() {

}

void FlySkyIBUS::begin(uint32_t ibus_baud) {
	ibus_config.state = DISCARD;
	ibus_config.last_millis = millis();
	ibus_config.ptr_buffer = NULL;
	ibus_config.packet_length = NULL;
	ibus_config.chksum = NULL;
	ibus_config.lchksum = NULL;

	FlySkyIBUSNext = FlySkyIBUSFirst;

	// ...try to get in sync with RMT someday
	hw_timer_t* timer = nullptr;
	timer = timerBegin(ibus_config.timer_id, IBUS_TIMER_DIVIDER, true);
	timerAttachInterrupt(timer, &onTimer, true);
	timerAlarmWrite(timer, 1000, true);  //1 ms
	timerAlarmEnable(timer);

	// ...first things first
	FlySkyIBUSFirst = this;
}

uint16_t FlySkyIBUS::readChannel(uint8_t channelNr) {
	if (channelNr < IBUS_MAX_CHANNELS) {
		return ibus_config.channel_data[channelNr];
	} else {
		return 0;
	}
}

uint8_t FlySkyIBUS::addSensor(uint8_t type, uint8_t len) {
	// add a sensor, return sensor number
	if (len != 2 && len != 4) len = 2;
		if (ibus_config.sensor_count < IBUS_SENSORS_MAX) {
			sensor_info* sensor_info_added = &sensors[ibus_config.sensor_count];
			sensor_info_added->sensorType = type;
			sensor_info_added->sensorLength = len;
			sensor_info_added->sensorValue = 0;
			ibus_config.sensor_count++;
	}

	return ibus_config.sensor_count;
}

void FlySkyIBUS::setSensorMeasurement(uint8_t adr, int32_t value) {
	if (adr <= ibus_config.sensor_count && adr > 0)
		sensors[adr - 1].sensorValue = value;
}

void FlySkyIBUS::process_ibus_data(void) {
	// ...call one after the other
	if (FlySkyIBUSNext) {
		FlySkyIBUSNext->process_ibus_data();
	}

	while (ibus_config.ibus_input->available() > 0) {
		// ...ibus pause
		uint32_t now = millis();
		if (now - ibus_config.last_millis >= IBUS_PAUSE) {
			ibus_config.state = GET_LENGTH;
		}

		ibus_config.last_millis = now;

		uint8_t ibus_raw = ibus_config.ibus_input->read();

		switch (ibus_config.state) {
			case GET_LENGTH:
				if (ibus_raw <= IBUS_LENGTH && ibus_raw > IBUS_OVERHEAD) {
					ibus_config.ptr_buffer = 0;
					ibus_config.packet_length = ibus_raw - IBUS_OVERHEAD;
					ibus_config.chksum = 0xFFFF - ibus_raw;
					ibus_config.state = GET_DATA;
				} else {
					ibus_config.state = DISCARD;
				}
				break;

			case GET_DATA:
				ibus_config.buffer[ibus_config.ptr_buffer++] = ibus_raw;
				ibus_config.chksum -= ibus_raw;

				if (ibus_config.ptr_buffer == ibus_config.packet_length) {
					ibus_config.state = GET_CHKSUML;
				}
				break;

			case GET_CHKSUML:
				ibus_config.lchksum = ibus_raw;
				ibus_config.state = GET_CHKSUMH;
				break;

			case GET_CHKSUMH:
				// ...checksum
				if (ibus_config.chksum == (ibus_raw << 8) + ibus_config.lchksum) {
					// ...packet valid
					uint8_t sensor_address = ibus_config.buffer[0] & 0x0f;

					// ...where will we start
					if (ibus_config.buffer[0] == IBUS_COMMAND40) {
						for (uint8_t i = 1; i < IBUS_MAX_CHANNELS * 2 + 1; i += 2) {
							ibus_config.channel_data[i / 2] = ibus_config.buffer[i] | (ibus_config.buffer[i + 1] << 8);
						}
						cnt_rec++;
					} else if (sensor_address <= ibus_config.sensor_count && sensor_address > 0 && ibus_config.packet_length == 1) {
						// all sensor data commands go here
						// we only process the len==1 commands (=message length is 4 bytes incl overhead) to prevent the case the
						// return messages from the UART TX port loop back to the RX port and are processed again. This is extra
						// precaution as it will also be prevented by the IBUS_PAUSE required
						sensor_info* sensor_data = &sensors[sensor_address - 1];
						// delayMicroseconds(100);

						switch (ibus_config.buffer[0] & 0xF0) {
							case IBUS_COMMAND_DISCOVER: // 0x80, discover sensor
								cnt_poll++;
								// echo discover command: 0x04, 0x81, 0x7A, 0xFF 
								ibus_config.ibus_input->write(0x04);
								ibus_config.ibus_input->write(IBUS_COMMAND_DISCOVER + sensor_address);
								ibus_config.chksum = 0xFFFF - (0x04 + IBUS_COMMAND_DISCOVER + sensor_address);
								break;

							case IBUS_COMMAND_TYPE: // 0x90, send sensor type
							  // echo sensortype command: 0x06 0x91 0x00 0x02 0x66 0xFF 
								ibus_config.ibus_input->write(0x06);
								ibus_config.ibus_input->write(IBUS_COMMAND_TYPE + sensor_address);
								ibus_config.ibus_input->write(sensor_data->sensorType);
								ibus_config.ibus_input->write(sensor_data->sensorLength);
								ibus_config.chksum = 0xFFFF - (0x06 + IBUS_COMMAND_TYPE + sensor_address + sensor_data->sensorType + sensor_data->sensorLength);
								break;

							case IBUS_COMMAND_VALUE: // 0xA0, send sensor data
								cnt_sensor++;
								uint8_t t;
								// echo sensor value command: 0x06 0x91 0x00 0x02 0x66 0xFF 
								ibus_config.ibus_input->write(t = 0x04 + sensor_data->sensorLength);
								ibus_config.chksum = 0xFFFF - t;
								ibus_config.ibus_input->write(t = IBUS_COMMAND_VALUE + sensor_address);
								ibus_config.chksum -= t;
								ibus_config.ibus_input->write(t = sensor_data->sensorValue & 0x0ff);
								ibus_config.chksum -= t;
								ibus_config.ibus_input->write(t = (sensor_data->sensorValue >> 8) & 0x0ff);
								ibus_config.chksum -= t;

								if (sensor_data->sensorLength == 4) {
									ibus_config.ibus_input->write(t = (sensor_data->sensorValue >> 16) & 0x0ff);
									ibus_config.chksum -= t;
									ibus_config.ibus_input->write(t = (sensor_data->sensorValue >> 24) & 0x0ff);
									ibus_config.chksum -= t;
								}
								break;

							default:
								sensor_address = 0; // unknown command, prevent sending chksum
								break;
						}

						if (sensor_address > 0) {
							ibus_config.ibus_input->write(ibus_config.chksum & 0x0ff);
							ibus_config.ibus_input->write(ibus_config.chksum >> 8);
						}
					}
				}

				ibus_config.state = DISCARD;
				break;

			case DISCARD:

			default:
				break;
		}
	}
}
