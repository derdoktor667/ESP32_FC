#pragma once

#include <Arduino.h>

class SystemState {
	public:
		SystemState();
		~SystemState();

		typedef enum system_state {
			RUNNING,
			ERROR,
			CONFIG,
		} system_state_t;

		typedef enum rx_type {
			NO_RX,
			PPM,
			IBUS,
			SBUS,
		} rx_type_t;

		typedef enum rx_mode {
			MODE_1 = 1,
			MODE_2,
			MODE_3,
			MODE_4,
		} rx_mode_t;
		
		typedef enum motor_type {
			NONE,
			PWM,
			ONESHOT,
			DSHOT,
		} motor_type_t;

		typedef enum flight_mode {
			LEVEL,
			STUNT,
			ACRO,
		} flight_mode_t;

		bool setup_done = false;

	private:
		const uint8_t RX_MAX_CHANNELS_PPM = 8;
		const uint8_t RX_MAX_CHANNELS_IBUS = 14;
};
