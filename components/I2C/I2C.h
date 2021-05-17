#pragma once

#include <stdint.h>
#include <sys/types.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

#include "components/fc_config.h"

class I2C {
	public:
	static const gpio_num_t DEFAULT_SDA_PIN = I2C1_SDA_PIN;
	static const gpio_num_t DEFAULT_CLK_PIN = I2C1_SCL_PIN;
	static const uint32_t DEFAULT_CLK_SPEED = I2C1_CLK_SPEED;

	I2C();
	~I2C();
	void init(uint8_t address, gpio_num_t sdaPin = DEFAULT_SDA_PIN, gpio_num_t sclPin = DEFAULT_CLK_PIN, uint32_t clkSpeed = DEFAULT_CLK_SPEED, i2c_port_t portNum = I2C_NUM_0, bool pullup = true);
	void beginTransaction();
	void endTransaction();
	uint8_t getAddress() const;
	
	void read(uint8_t* bytes, size_t length, bool ack = true);
	void read(uint8_t* byte, bool ack = true);
	void scan();
	void setAddress(uint8_t address);
	bool slavePresent(uint8_t address);
	void start();
	void stop();
	void write(uint8_t byte, bool ack = true);
	void write(uint8_t* bytes, size_t length, bool ack = true);

	private:
	uint8_t				i2c_client_address;
	i2c_cmd_handle_t	i2c_cmd;
	bool				i2c_direction_known;
	gpio_num_t			i2c_sda_pin;
	gpio_num_t			i2c_scl_pin;
	i2c_port_t			i2c_port_num;
	uint32_t			i2c_clk_speed;
	static bool			i2c_driver_Installed;

};
