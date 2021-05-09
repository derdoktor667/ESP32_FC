// Author:	derdoktor667
//

#pragma once

#include <Arduino.h>
#include <driver/i2c.h>
#include <esp32-hal-i2c.h>
#include <Wire.h>

class I2C {
	public:
		I2C(uint8_t bus_nr = I2C_NUM_0);
		I2C(uint8_t address, gpio_num_t sdaPin = DEFAULT_SDA_PIN, gpio_num_t sclPin = DEFAULT_CLK_PIN, uint32_t clkSpeed = DEFAULT_CLK_SPEED, i2c_port_t portNum = I2C_NUM_0, bool pullup = true);
		~I2C();

		static const gpio_num_t DEFAULT_SDA_PIN = GPIO_NUM_21;
		static const gpio_num_t DEFAULT_CLK_PIN = GPIO_NUM_22;
		static const uint32_t DEFAULT_CLK_SPEED = 1000000;

		void init();
		void init(uint8_t address, gpio_num_t sdaPin = DEFAULT_SDA_PIN, gpio_num_t sclPin = DEFAULT_CLK_PIN, uint32_t clkSpeed = DEFAULT_CLK_SPEED, i2c_port_t portNum = I2C_NUM_0, bool pullup = true);
			
		void beginTransaction();
		void endTransaction();
	
		void write(uint8_t byte, bool ack = true);
		void write(uint8_t* bytes, size_t length, bool ack = true);
		void read(uint8_t* byte, bool ack = true);
		void read(uint8_t* bytes, size_t length, bool ack = true);
		
		void start();
		void stop();
		void scan();
		
		const uint8_t getAddress();
		void setAddress(uint8_t address);

		bool slavePresent(uint8_t address);

	private:
		typedef struct I2C_setup_s {
			uint8_t				address;
			i2c_cmd_handle_t	cmd;
			gpio_num_t			sdaPin;
			gpio_num_t			sclPin;
			i2c_port_t			portNum;
			uint32_t			speed;
			bool				directionKnown;
			uint8_t				clk_flags;
		};

		I2C_setup_s I2C_setup_t;
		i2c_config_t conf;
};
