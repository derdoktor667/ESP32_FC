
#include <Arduino.h>
#include "MPU6050.h"
#include "MP6050_registers.h"

MPU6050::MPU6050(gpio_num_t sda, gpio_num_t scl, int clk_speed) {}

MPU6050::~MPU6050() {}

void MPU6050::writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
	Wire.beginTransmission(address);
	Wire.write(subAddress);
	Wire.write(data);
	Wire.endTransmission();
}

uint8_t MPU6050::readByte(uint8_t address, uint8_t subAddress) {
	uint8_t data = NULL;
	Wire.beginTransmission(address);
	Wire.write(subAddress);
	Wire.endTransmission(false);
	Wire.requestFrom(address, (uint8_t)1);  // Read one byte from slave register address
	data = Wire.read();
	return data;
}

void MPU6050::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
	uint8_t i = NULL;

	Wire.beginTransmission(address);
	Wire.write(subAddress);
	Wire.endTransmission(false);
	Wire.requestFrom(address, count);
	
	while (Wire.available()) {
		dest[i++] = Wire.read();
	} // Put read results in the Rx buffer
}

void MPU6050::begin() {
	// get stable time source
	writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
	writeByte(MPU6050_ADDRESS, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t tmp_byte = readByte(MPU6050_ADDRESS, GYRO_CONFIG);
	writeByte(MPU6050_ADDRESS, GYRO_CONFIG, tmp_byte & ~0xE0); // Clear self-test bits [7:5]
	writeByte(MPU6050_ADDRESS, GYRO_CONFIG, tmp_byte & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU6050_ADDRESS, GYRO_CONFIG, tmp_byte | gyro_resolution << 3); // Set full scale range for the gyro

	// Set accelerometer configuration
	tmp_byte = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
	writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, tmp_byte & ~0xE0); // Clear self-test bits [7:5]
	writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, tmp_byte & ~0x18); // Clear AFS bits [4:3]
	writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, tmp_byte | acc_resolution << 3); // Set full scale range for the accelerometer

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

void MPU6050::calibrate(float* axis1, float* axis2) {
	uint8_t raw_data[12] = { };
	uint16_t packet_count;
	uint16_t fifo_count;
	int32_t gyro_bias[3] = { };
	int32_t accel_bias[3] = { };

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
	writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);

	// Configure device for bias calculation
	writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);
	writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);
	writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);
	writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00);
	writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);
	writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyro_sens = 131;   // = 131 LSB/degrees/sec
	uint16_t  acc_sens = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);
	writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);
	delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00); 
	readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &raw_data[0]);

	fifo_count = ((uint16_t)raw_data[0] << 8) | raw_data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

	for (int i = 0; i < packet_count; i++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &raw_data[0]);
		accel_temp[0] = (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
		accel_temp[2] = (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

		gyro_temp[0] = (int16_t)(((int16_t)raw_data[6] << 8) | raw_data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)raw_data[8] << 8) | raw_data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)raw_data[10] << 8) | raw_data[11]);

		accel_bias[0] += (int32_t)accel_temp[0];
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];

		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];

	}
	accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;

	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	if (accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t)acc_sens; // Remove gravity from the z-axis accelerometer bias calculation
	} else {
		accel_bias[2] += (int32_t)acc_sens;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	raw_data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	raw_data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	raw_data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	raw_data[3] = (-gyro_bias[1] / 4) & 0xFF;
	raw_data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	raw_data[5] = (-gyro_bias[2] / 4) & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, raw_data[0]);// might not be supported in MPU6050
	writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, raw_data[1]);
	writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, raw_data[2]);
	writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, raw_data[3]);
	writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, raw_data[4]);
	writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, raw_data[5]);

	axis1[0] = (float)gyro_bias[0] / (float)gyro_sens; // construct gyro bias in deg/s for later manual subtraction
	axis1[1] = (float)gyro_bias[1] / (float)gyro_sens;
	axis1[2] = (float)gyro_bias[2] / (float)gyro_sens;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &raw_data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t)((int16_t)raw_data[0] << 8) | raw_data[1];
	readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &raw_data[0]);
	accel_bias_reg[1] = (int16_t)((int16_t)raw_data[0] << 8) | raw_data[1];
	readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &raw_data[0]);
	accel_bias_reg[2] = (int16_t)((int16_t)raw_data[0] << 8) | raw_data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (int i = 0; i < 3; i++) {
		if (accel_bias_reg[i] & mask) mask_bit[i] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	raw_data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	raw_data[1] = (accel_bias_reg[0]) & 0xFF;
	raw_data[1] = raw_data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	raw_data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	raw_data[3] = (accel_bias_reg[1]) & 0xFF;
	raw_data[3] = raw_data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	raw_data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	raw_data[5] = (accel_bias_reg[2]) & 0xFF;
	raw_data[5] = raw_data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Push accelerometer biases to hardware registers
	writeByte(MPU6050_ADDRESS, XA_OFFSET_H, raw_data[0]); // might not be supported in MPU6050
	writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, raw_data[1]);
	writeByte(MPU6050_ADDRESS, YA_OFFSET_H, raw_data[2]);
	writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, raw_data[3]);
	writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, raw_data[4]);
	writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, raw_data[5]);

	// Output scaled accelerometer biases for manual subtraction in the main program
	axis2[0] = (float)accel_bias[0] / (float)acc_sens;
	axis2[1] = (float)accel_bias[1] / (float)acc_sens;
	axis2[2] = (float)accel_bias[2] / (float)acc_sens;
}

float MPU6050::get_acc_resolution() {
	switch (acc_resolution) {
		case AFS_2G:
			return 2.0 / 32768.0;
			break;

		case AFS_4G:
			return 4.0 / 32768.0;
			break;

		case AFS_8G:
			return 8.0 / 32768.0;
			break;

		case AFS_16G:
			return 16.0 / 32768.0;
			break;
	}
}

float MPU6050::get_gyro_resolution() {
	switch (gyro_resolution) {
		case GFS_250DPS:
			return 250.0 / 32768.0;
			break;

		case GFS_500DPS:
			return 500.0 / 32768.0;
			break;

		case GFS_1000DPS:
			return 1000.0 / 32768.0;
			break;

		case GFS_2000DPS:
			return 2000.0 / 32768.0;
			break;
	}
}

void MPU6050::read_Acc_Data(int16_t* destination) {
	uint8_t acc_raw[6] = { };
	readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &acc_raw[0]);
	destination[0] = (int16_t)((acc_raw[0] << 8) | acc_raw[1]);
	destination[1] = (int16_t)((acc_raw[2] << 8) | acc_raw[3]);
	destination[2] = (int16_t)((acc_raw[4] << 8) | acc_raw[5]);
}

void MPU6050::read_Gyro_Data(int16_t* destination) {
	uint8_t gyro_raw[6] = { };
	readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &gyro_raw[0]);
	destination[0] = (int16_t)((gyro_raw[0] << 8) | gyro_raw[1]);
	destination[1] = (int16_t)((gyro_raw[2] << 8) | gyro_raw[3]);
	destination[2] = (int16_t)((gyro_raw[4] << 8) | gyro_raw[5]);
}

int16_t MPU6050::read_Temperature_Data() {
	uint8_t temperature_raw[2] = { };
	readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &temperature_raw[0]);
	return ((int16_t)temperature_raw[0]) << 8 | temperature_raw[1];
}
