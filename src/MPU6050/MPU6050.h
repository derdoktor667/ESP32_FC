#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "MP6050_registers.h"
#include "../../fc_config.h"

constexpr auto MPU6050_ADDRESS = 0x68;

enum acc_resolution_e {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
} acc_resolution;

enum gyro_resolution_e {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
} gyro_resolution;

class MPU6050 {
	public:
	MPU6050(gpio_num_t sda, gpio_num_t scl, int clk_speed);
	~MPU6050();

	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
	uint8_t readByte(uint8_t address, uint8_t subAddress);
	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);

	void begin();
	void calibrate(float* axis1, float* axis2);
	float get_acc_resolution();
	float get_gyro_resolution();
	void read_Acc_Data(int16_t* destination);
	void read_Gyro_Data(int16_t* destination);
	int16_t read_Temperature_Data();

	private:
	int16_t acc_raw_x;
	int16_t acc_raw_y;
	int16_t acc_raw_z;
	int16_t gyro_raw_x; 
	int16_t gyro_raw_y;
	int16_t gyro_raw_z;

};

