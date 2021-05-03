// Author:	derdoktor667
//

#pragma once

#include <Arduino.h>
#include <string>
#include "driver/rmt.h"
#include "freertos/task.h"

constexpr auto DSHOT_CLK_DIVIDER = 8; // ...slow down RMT clock to 10Mhz
constexpr auto DSHOT_PACKET_LENGTH = 17;

constexpr auto DSHOT_THROTTLE_MIN = 48;
constexpr auto DSHOT_THROTTLE_MAX = 2047;

constexpr auto DSHOT_PAUSE = (2000 * DSHOT_CLK_DIVIDER); // ...21bit is recommended, just for sure some more time
constexpr auto DSHOT_PAUSE_BIT = (DSHOT_PACKET_LENGTH - 1);
constexpr auto DSHOT_ARM_DELAY = (5000 / portTICK_PERIOD_MS);

// ...convert ESP32 CPU cycles to RMT device cycles, for info only
constexpr auto F_CPU_RMT = 80000000L;
constexpr auto RMT_CYCLES_PER_SEC = (F_CPU_RMT / DSHOT_CLK_DIVIDER);
constexpr auto RMT_CYCLES_PER_ESP_CYCLE = (F_CPU / RMT_CYCLES_PER_SEC);

typedef enum mode {
	DSHOT_OFF,
	DSHOT150,
	DSHOT300,
	DSHOT600,
	DSHOT1200
} dshot_mode_t;

typedef struct dshot_packet_s {
	uint16_t payload;
	bool telemetry;
} dshot_packet_t;

class DShotRMT {
	public:
		DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel, bool wait = false);
		~DShotRMT();

		void init(dshot_mode_t mode);
		void sendThrottle(uint16_t throttle);
		void setReversed(bool reversed = false);
		void beep();

		virtual String get_dshot_mode();
		virtual uint8_t get_dshot_clock_div();

		bool isArmed = false;
		dshot_packet_t throttle_packet;

	private:
		typedef String dshot_name_t;

		typedef struct dshot_config_s {
			dshot_mode_t mode;
			dshot_name_t name_str;
			gpio_num_t gpio_num;
			uint8_t pin_num;
			rmt_channel_t rmt_channel;
			uint8_t mem_block_num;
			uint8_t ticks_per_packet;
			uint8_t clk_div;
			uint8_t ticks_zero_high;
			uint8_t ticks_zero_low;
			uint8_t ticks_one_high;
			uint8_t ticks_one_low;
		} dshot_config_t;

		// source: https://github.com/bitdump/BLHeli/blob/master/BLHeli_S%20SiLabs/Dshotprog%20spec%20BLHeli_S.txt
		enum dshot_cmd_t {
			DIGITAL_CMD_MOTOR_STOP,							// Currently not implemented
			DIGITAL_CMD_BEEP1, 								// Wait at least length of beep (380ms) before next command
			DIGITAL_CMD_BEEP2, 								// Wait at least length of beep (380ms) before next command
			DIGITAL_CMD_BEEP3, 								// Wait at least length of beep (400ms) before next command
			DIGITAL_CMD_BEEP4, 								// Wait at least length of beep (400ms) before next command
			DIGITAL_CMD_BEEP5, 								// Wait at least length of beep (400ms) before next command
			DIGITAL_CMD_ESC_INFO,  							// Wait at least 12ms before next command
			DIGITAL_CMD_SPIN_DIRECTION_1, 					// Currently not implemented
			DIGITAL_CMD_SPIN_DIRECTION_2, 					// Need 6x, no wait required
			DIGITAL_CMD_3D_MODE_OFF, 						// Need 6x, no wait required
			DIGITAL_CMD_3D_MODE_ON,  						// Need 6x, no wait required
			DIGITAL_CMD_SETTINGS_REQUEST,  					// Currently not implemented
			DIGITAL_CMD_SAVE_SETTINGS,  					// Need 6x, wait at least 12ms before next command
			DIGITAL_CMD_SPIN_DIRECTION_NORMAL = 20, 	    // Need 6x, no wait required
			DIGITAL_CMD_SPIN_DIRECTION_REVERSED, 			// Need 6x, no wait required
			DIGITAL_CMD_LED0_ON, 							// No wait required
			DIGITAL_CMD_LED1_ON, 							// No wait required
			DIGITAL_CMD_LED2_ON, 							// No wait required
			DIGITAL_CMD_LED3_ON, 							// No wait required
			DIGITAL_CMD_LED0_OFF, 							// No wait required
			DIGITAL_CMD_LED1_OFF, 							// No wait required
			DIGITAL_CMD_LED2_OFF, 							// No wait required
			DIGITAL_CMD_LED3_OFF, 							// No wait required
			DSHOT_CMD_MAX = 47
		};

		uint8_t checksum(uint16_t data);

		dshot_packet_t prepareDShotPacket(uint16_t throttle, bool telemetry = false);

		void setData(uint16_t data);
		void writeData(uint16_t data, bool wait);
		void writePacket(dshot_packet_t packet, bool wait = false);
		void repeatPacket(dshot_packet_t packet, int n);
		void repeatPacketTicks(dshot_packet_t packet, TickType_t ticks);
				
		rmt_item32_t* dshot_command = new rmt_item32_t[DSHOT_PACKET_LENGTH];
		dshot_config_t dshot_config;
		rmt_config_t config;
};
