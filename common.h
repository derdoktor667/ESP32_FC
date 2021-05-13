// Author:	derdoktor667
//

#pragma once

#include <Arduino.h>
#include <esp_task_wdt.h>

#ifndef __cplusplus
#define bool int
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

hw_timer_t* timer = NULL;

// ...and there was light
template<typename T>
constexpr int MHZ_TO_HZ(T x) {	return ((x) * 1000000); }

template<typename T>
constexpr int ARRAY_SIZE(T x) {return (sizeof(x) / sizeof(x[0])); }

//
struct hardware_info {
	const char* chipModel = ESP.getChipModel();
	int chipMhz = ESP.getCpuFreqMHz();
	int chipCores = ESP.getChipCores();
	int flashChipSpeed = ESP.getFlashChipSpeed();
	int flashChipSize = ESP.getFlashChipSize();
} hardware_info_s;
