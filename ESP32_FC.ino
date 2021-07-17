/*
 Name:		ESP32_ESC.ino
 Created:	20.03.2021 00:49:15
 Author:	derdoktor667
*/

#include <Arduino.h>

// ...the good parts
#include "src/fc_config.h"
#include "src/I2Cdev/I2Cdev.h"
#include "src/FlySkyIBUS/FlySkyIBUS.h"
#include "src/DShotRMT/DShotRMT.h"
#include "src/SystemState/SystemState.h"

// ...better usb port naming
HardwareSerial &USB_Serial = Serial;
constexpr auto USB_SERIAL_BAUD = 115200;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

// ...is Bluetooth enabled, we will need it later???
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
	#error ...Bluetooth is not enabled!!!
#endif

constexpr auto MOTOR_1 = GPIO_NUM_4;
constexpr auto MOTOR_2 = GPIO_NUM_0;
constexpr auto MOTOR_3 = GPIO_NUM_27;
constexpr auto MOTOR_4 = GPIO_NUM_26;

// ...hardware init
FlySkyIBUS ibus;

DShotRMT dshot_1(MOTOR_1, RMT_CHANNEL_0);
DShotRMT dshot_2(MOTOR_2, RMT_CHANNEL_1);
DShotRMT dshot_3(MOTOR_3, RMT_CHANNEL_2);
DShotRMT dshot_4(MOTOR_4, RMT_CHANNEL_3);

volatile auto throttle_value = 0;

firmware_info_s firmware_info;

void setup() {
	// ...always start the onboard usb support
	USB_Serial.begin(USB_SERIAL_BAUD);

	// IBUS will be read on Serial2 on ESP32 by default
	ibus.begin();

	// ...select the DSHOT Mode
	dshot_1.begin(DSHOT600);
	dshot_2.begin(DSHOT600);
	dshot_3.begin(DSHOT600);
	dshot_4.begin(DSHOT600);

    USB_Serial.println(firmware_info.device_name);
    USB_Serial.println(F_CPU);
    USB_Serial.println(APB_CLK_FREQ);
}

void loop() {
	read_SerialThrottle();

	dshot_1.send_dshot_value(throttle_value);
	dshot_2.send_dshot_value(throttle_value);
	dshot_3.send_dshot_value(throttle_value);
	dshot_4.send_dshot_value(throttle_value);

	// USB_Serial.println(throttle_value);
}

void read_SerialThrottle() {
	if (USB_Serial.available() > 0) {
		auto throttle_input = (USB_Serial.readStringUntil('\n')).toInt();
		throttle_value = throttle_input;
	}
}

// void update_throttle_reading() {
//	auto rc_readings_All = ibus.get_IBUS_Channels();
//	throttle_value = rc_readings_All[THROTTLE];
// }
