// Author:	derdoktor667
//

#pragma once

#include <Arduino.h>

#ifndef __cplusplus
#define true !(0)
#define false 0
#endif

#ifdef SERIAL
HardwareSerial* USB_Serial = &Serial;
constexpr auto USB_SERIAL_BAUD = 115200;
#endif // SERIAL

// ...ESP32 DEV Kit Hardware I2C pins
constexpr gpio_num_t I2C1_SDA_PIN = GPIO_NUM_21;
constexpr gpio_num_t I2C1_SCL_PIN = GPIO_NUM_22;

// ...ESP32 UART Hardware pins
constexpr auto UART_1_BAUD = 115200;
constexpr auto UART_1_TX_PIN = 16;
constexpr auto UART_1_RX_PIN = 17;

typedef enum UART_Num{
	NONE,
	UART_1,
	UART_2,
	UART_3,
};

// ...some calculations
template<typename T>
constexpr int MHZ_TO_HZ(T x) {	return ((x) * 1000000); }

template<typename T>
constexpr int ARRAY_SIZE(T x) {return (sizeof(x) / sizeof(x[0])); }

// ...ESP32 Info
struct hardware_info {
	const char* chipModel = ESP.getChipModel();
	uint32_t chipMhz = ESP.getCpuFreqMHz();
	uint8_t chipCores = ESP.getChipCores();
	uint32_t flashChipSpeed = ESP.getFlashChipSpeed();
	uint32_t flashChipSize = ESP.getFlashChipSize();
} hardware_info_s;
