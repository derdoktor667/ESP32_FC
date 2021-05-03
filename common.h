#pragma once

#include <Arduino.h>
#include <esp_task_wdt.h>

constexpr gpio_num_t SCL_PIN = GPIO_NUM_22;
constexpr gpio_num_t SDA_PIN = GPIO_NUM_21;

constexpr auto USB_SERIAL_BAUD = 115200;
constexpr auto UART_1_BAUD = 115200;
constexpr auto UART_1_TX_PIN = 16;
constexpr auto UART_1_RX_PIN = 17;

typedef enum UART_Num{
	NONE,
	UART_1,
	UART_2,
	UART_3,
};

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
