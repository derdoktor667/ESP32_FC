#include <driver/gpio.h>uii
#include <driver/i2c.h>
#include <stdint.h>
#include <sys/types.h>22
#include "I2C.h"

I2C::I2C() {
	i2c_direction_known = false;
	i2c_client_address = 0;
	i2c_cmd = 0;
	i2c_sda_pin = DEFAULT_SDA_PIN;
	i2c_scl_pin = DEFAULT_CLK_PIN;
	i2c_port_num = I2C_NUM_0;
}

I2C::~I2C() {
	if (i2c_driver_Installed) {
		i2c_driver_delete(i2c_port_num);
	}
}

void I2C::init(uint8_t address, gpio_num_t sdaPin, gpio_num_t sclPin, uint32_t clockSpeed, i2c_port_t portNum, bool pullup) {
	i2c_port_num = portNum;
	i2c_sda_pin = sdaPin;
	i2c_scl_pin = sclPin;
	i2c_client_address = address;
	i2c_clk_speed = clockSpeed;

	i2c_config_t conf = {};
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = sdaPin;
	conf.scl_io_num = sclPin;
	conf.sda_pullup_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.scl_pullup_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = clockSpeed;

	i2c_param_config(i2c_port_num, &conf);

	if (!i2c_driver_Installed) {
		i2c_driver_install(i2c_port_num, I2C_MODE_MASTER, 0, 0, 0);
		i2c_driver_Installed = true;
	}
}

void I2C::beginTransaction() {
	i2c_cmd = i2c_cmd_link_create();
	i2c_master_start(i2c_cmd);
	i2c_direction_known = false;
}

void I2C::endTransaction() {
	i2c_master_stop(i2c_cmd);
	i2c_master_cmd_begin(i2c_port_num, i2c_cmd, 0);
	i2c_cmd_link_delete(i2c_cmd);
	i2c_direction_known = false;
}

uint8_t I2C::getAddress() const {
	return i2c_client_address;
}

void I2C::read(uint8_t* bytes, size_t length, bool ack) {
	if (!i2c_direction_known) {
		i2c_direction_known = true;
		i2c_master_write_byte(i2c_cmd, (i2c_client_address << 1) | I2C_MASTER_READ, !ack);
	}
	i2c_master_read(i2c_cmd, bytes, length, ack ? I2C_MASTER_ACK : I2C_MASTER_NACK);
}

void I2C::read(uint8_t* byte, bool ack) {
	if (!i2c_direction_known) {
		i2c_direction_known = true;
		i2c_master_write_byte(i2c_cmd, (i2c_client_address << 1) | I2C_MASTER_READ, !ack);
	}
	i2c_master_read_byte(i2c_cmd, byte, ack ? I2C_MASTER_ACK : I2C_MASTER_NACK);
}

void I2C::scan() {
	printf("Data Pin: %d, Clock Pin: %d\n", this->i2c_sda_pin, this->i2c_scl_pin);
	printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
	printf("00:         ");
	for (uint8_t i = 3; i < 0x78; i++) {
		if (i % 16 == 0) {
			printf("\n%.2x:", i);
		}
		if (slavePresent(i)) {
			printf(" %.2x", i);
		}
		else {
			printf(" --");
		}
	}
	printf("\n");
}

void I2C::setAddress(uint8_t address) {
	this->i2c_client_address = address;
}

bool I2C::slavePresent(uint8_t address) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
	i2c_master_stop(cmd);

	bool gotAnswer = i2c_master_cmd_begin(i2c_port_num, cmd, 0);
	i2c_cmd_link_delete(cmd);
	return gotAnswer == false;  // Return true if the slave is present and false otherwise.
} 

void I2C::start() {
	i2c_master_start(i2c_cmd);
	i2c_direction_known = false;
}

void I2C::stop() {
	i2c_master_stop(i2c_cmd);
	i2c_direction_known = false;
}
void I2C::write(uint8_t byte, bool ack) {
	if (!i2c_direction_known) {
		i2c_direction_known = true;
		i2c_master_write_byte(i2c_cmd, (i2c_client_address << 1) | I2C_MASTER_WRITE, !ack);
	}

	i2c_master_write_byte(i2c_cmd, byte, !ack);
}

void I2C::write(uint8_t* bytes, size_t length, bool ack) {
	if (!i2c_direction_known) {
		i2c_direction_known = true;
		i2c_master_write_byte(i2c_cmd, (i2c_client_address << 1) | I2C_MASTER_WRITE, !ack);
	}
	
	i2c_master_write(i2c_cmd, bytes, length, !ack);
}
