#pragma once

//
//
//

#include <Arduino.h>
#include <string>
#include "driver/rmt.h"
#include "freertos/task.h"

constexpr auto DSHOT_PACKET_LENGTH = 17;

constexpr auto DSHOT_THROTTLE_MIN = 48;
constexpr auto DSHOT_THROTTLE_MAX = 2047;

constexpr auto DSHOT_PAUSE = 21; // 21bit is recommended
constexpr auto DSHOT_ARM_DELAY = (5000 / portTICK_PERIOD_MS);

typedef enum dshot_mode {
	DSHOT_OFF,
	DSHOT150,
	DSHOT300,
	DSHOT600,
	DSHOT1200
} dshot_mode_t;

class DShotRMT {
	public:
		DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel, bool wait = true);
		~DShotRMT();

		void init(dshot_mode_t mode);
		void sendThrottle(uint16_t throttle);
		void setReversed(bool reversed);
		void beep();

		String get_dshot_mode();

	private:
		typedef String dshot_name_t;

		typedef struct dshot_config_s {
			dshot_mode_t dshot_mode;
			dshot_name_t dshot_name;
			gpio_num_t dshot_gpio;
			uint8_t dshot_pin;
			rmt_channel_t dshot_rmtChannel;
			uint8_t dshot_mem_block_num;
			uint8_t dshot_packet_ticks;
			uint8_t dshot_clk_div;
			uint8_t dshot_t0h;
			uint8_t dshot_t0l;
			uint8_t dshot_t1h;
			uint8_t dshot_t1l;
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
	
		struct dshot_packet_t {
			uint16_t payload;
			bool telemetry;
		};

		volatile uint8_t checksum(uint16_t data);

		void setData(uint16_t data);
		void writeData(uint16_t data, bool wait);
		void writePacket(dshot_packet_t packet, bool wait);
		void repeatPacket(dshot_packet_t packet, int n);
		void repeatPacketTicks(dshot_packet_t packet, TickType_t ticks);
				
		rmt_item32_t _dshotCmd[DSHOT_PACKET_LENGTH];
		dshot_config_t dshot_config;
		rmt_config_t config;
};
