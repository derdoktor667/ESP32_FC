//
// Name:		ESP32_ESC.ino
// Created: 	20.03.2021 00:49:15
// Author:  	derdoktor667
//

#pragma once

#include <Arduino.h>


// ...Version Info
constexpr auto VERSION_MAJOR = 0;
constexpr auto VERSION_MINOR = 1;
constexpr auto VERSION_REV = 1;

// ...I2C_CLK_MODE default and fast
constexpr auto I2C_DEFAULT_SPEED = 100000;
constexpr auto I2C_FAST_SPEED = 400000;

// ...ESP32 DEV Kit Hardware I2C pins
constexpr auto I2C1_SDA_PIN = GPIO_NUM_21;
constexpr auto I2C1_SCL_PIN = GPIO_NUM_22;
constexpr auto I2C1_CLK_SPEED = I2C_FAST_SPEED;

// ...ESP32 UART Hardware pins
constexpr auto UART_1_BAUD = 115200;
constexpr auto UART_1_TX_PIN = 16;
constexpr auto UART_1_RX_PIN = 17;

typedef enum uart_num_e {
	NONE,
	UART_1,
	UART_2,
	UART_3,
} uart_num_t;

// ...decode rc input as MODE 2
enum rx_mode_2_e {
	AILERON,
	ELEVATOR,
	THROTTLE,
	RUDDER,
};

// ...add an "ARMED" channel and up to 9 AUX channels
enum rx_aux_channels_e {
	ARMED = 5,
	AUX1,
	AUX2,
	AUX3,
	AUX4,
	AUX5,
	AUX6,
	AUX7,
	AUX8,
	AUX9,
};

// ...return Firmware Info asa pointer to firmware_info
typedef struct firmware_info_s {
	const uint8_t version_major = VERSION_MAJOR;
	const uint8_t version_minor = VERSION_MINOR;
	const uint8_t version_rev = VERSION_REV;
	const char* device_name = ESP.getChipModel();
} firmware_info_t;

firmware_info_t* get_Firmware_Info();
