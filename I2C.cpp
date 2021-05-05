#include "I2C.h"

#include <Arduino.h>
#include <driver/i2c.h>
#include <esp32-hal-i2c.h>

I2C::I2C() {
	I2C_setup_t.address = 0;
	I2C_setup_t.cmd = 0;
	I2C_setup_t.sdaPin = DEFAULT_SDA_PIN;
	I2C_setup_t.sclPin = DEFAULT_CLK_PIN;
	I2C_setup_t.portNum = I2C_NUM_0;
	I2C_setup_t.speed = 100000;
	I2C_setup_t.directionKnown = false;
	I2C_setup_t.clk_flags = (1 << 0);
}

I2C::I2C(uint8_t address, gpio_num_t sdaPin, gpio_num_t sclPin, uint32_t clkSpeed, i2c_port_t portNum, bool pullup) {
	I2C_setup_t.address = address;
	I2C_setup_t.portNum = portNum;
	I2C_setup_t.sdaPin = sdaPin;
	I2C_setup_t.sclPin = sclPin;
	I2C_setup_t.speed = clkSpeed;

	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_setup_t.sdaPin;
	conf.scl_io_num = I2C_setup_t.sclPin;
	conf.sda_pullup_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.scl_pullup_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = I2C_setup_t.speed;
}

// ...no comment
I2C::~I2C() {

}

void I2C::init() {
	i2c_param_config(I2C_setup_t.portNum, &conf);
	i2c_driver_install(I2C_setup_t.portNum, I2C_MODE_MASTER, 0, 0, 0);
}

void I2C::init(uint8_t address, gpio_num_t sdaPin, gpio_num_t sclPin, uint32_t clkSpeed, i2c_port_t portNum, bool pullup) {
	I2C_setup_t.address = address;
	I2C_setup_t.portNum = portNum;
	I2C_setup_t.sdaPin = sdaPin;
	I2C_setup_t.sclPin = sclPin;
	I2C_setup_t.speed = clkSpeed;


	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_setup_t.sdaPin;
	conf.scl_io_num = I2C_setup_t.sclPin;
	conf.sda_pullup_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.scl_pullup_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = I2C_setup_t.speed;

	i2c_param_config(I2C_setup_t.portNum, &conf);
	i2c_driver_install(I2C_setup_t.portNum, I2C_MODE_MASTER, 0, 0, 0);
}

void I2C::beginTransaction() {
	I2C_setup_t.cmd = i2c_cmd_link_create();
	i2c_master_start(I2C_setup_t.cmd);
	I2C_setup_t.directionKnown = false;
}

void I2C::endTransaction() {
	i2c_master_stop(I2C_setup_t.cmd);

	i2c_master_cmd_begin(I2C_setup_t.portNum, I2C_setup_t.cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(I2C_setup_t.cmd);
	I2C_setup_t.directionKnown = false;
}

void I2C::write(uint8_t byte, bool ack) {
	if (!I2C_setup_t.directionKnown) {
		I2C_setup_t.directionKnown = true;
		i2c_master_write_byte(I2C_setup_t.cmd, (I2C_setup_t.address << 1) | I2C_MASTER_WRITE, !ack);
	}

	i2c_master_write_byte(I2C_setup_t.cmd, byte, !ack);
}

void I2C::write(uint8_t* bytes, size_t length, bool ack) {
	if (!I2C_setup_t.directionKnown) {
		I2C_setup_t.directionKnown = true;
		i2c_master_write_byte(I2C_setup_t.cmd, (I2C_setup_t.address << 1) | I2C_MASTER_WRITE, !ack);
	}

	i2c_master_write(I2C_setup_t.cmd, bytes, length, !ack);
}

void I2C::read(uint8_t* byte, bool ack) {
	if (!I2C_setup_t.directionKnown) {
		I2C_setup_t.directionKnown = true;
		i2c_master_write_byte(I2C_setup_t.cmd, (I2C_setup_t.address << 1) | I2C_MASTER_READ, !ack);
	}

	i2c_master_read_byte(I2C_setup_t.cmd, byte, ack ? I2C_MASTER_ACK : I2C_MASTER_NACK);
}

void I2C::read(uint8_t* bytes, size_t length, bool ack) {
	if (!I2C_setup_t.directionKnown) {
		I2C_setup_t.directionKnown = true;
		i2c_master_write_byte(I2C_setup_t.cmd, (I2C_setup_t.address << 1) | I2C_MASTER_READ, !ack);
	}

	i2c_master_read(I2C_setup_t.cmd, bytes, length, ack ? I2C_MASTER_ACK : I2C_MASTER_NACK);
}

void I2C::start() {
	i2c_master_start(I2C_setup_t.cmd);
	I2C_setup_t.directionKnown = false;
}

void I2C::stop() {
	i2c_master_stop(I2C_setup_t.cmd);
	I2C_setup_t.directionKnown = false;
}

void I2C::scan() {
	printf("\n");
	printf("Data Pin: %d, Clock Pin: %d\n", I2C_setup_t.sdaPin, I2C_setup_t.sclPin);
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");

	for (int i = 3; i < 0x78; i++) {
		if (i % 16 == 0) {
			printf("\n%.2x:", i);
		}

		if (slavePresent(i)) {
			printf(" %.2x", i);
		} else {
			printf(" --");
		}
	}
	printf("\n");
}

const uint8_t I2C::getAddress() {
	return I2C_setup_t.address;
}

void I2C::setAddress(uint8_t address) {
	I2C_setup_t.address = address;
}

bool I2C::slavePresent(uint8_t address) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	i2c_master_stop(cmd);

	bool foundSome = i2c_master_cmd_begin(I2C_setup_t.portNum, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	return foundSome == 0;  // Return true if the slave is present and false otherwise.
}
