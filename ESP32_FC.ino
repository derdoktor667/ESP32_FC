/*
 Name:		ESP32_ESC.ino
 Created:	20.03.2021 00:49:15
 Author:	derdoktor667
*/

#include <Arduino.h>
#include <Preferences.h>
#include "common.h"
#include "DShotRMT.h"

constexpr auto MOTOR_1 = GPIO_NUM_16;
constexpr auto MOTOR_2 = GPIO_NUM_17;
constexpr auto MOTOR_3 = GPIO_NUM_26;
constexpr auto MOTOR_4 = GPIO_NUM_27;

// ...hardware init
HardwareSerial* USB_Serial = &Serial;
DShotRMT dshot_1(MOTOR_1, RMT_CHANNEL_0);
DShotRMT dshot_2(MOTOR_2, RMT_CHANNEL_1);
DShotRMT dshot_3(MOTOR_3, RMT_CHANNEL_2);
DShotRMT dshot_4(MOTOR_4, RMT_CHANNEL_3);

// the setup function runs once when you press reset or power the board
void setup() {
	USB_Serial->begin(USB_SERIAL_BAUD);
	USB_Serial->setTimeout(portTICK_PERIOD_MS);

	// ...select the DSHOT Mode
	dshot_1.init(DSHOT300);
	dshot_2.init(DSHOT300);
	dshot_3.init(DSHOT300);
	dshot_4.init(DSHOT300);
}

volatile auto x = 48;

// the loop function runs over and over again until power down or reset
void loop() {
	readSerialThrottle();
	sendThrottleSynced(x);
}

void readSerialThrottle() {
	if (USB_Serial->available() > 0) {
		String str = USB_Serial->readStringUntil('\n');
		x = str.toInt();
	}
}

void sendThrottleSynced(uint16_t throttle_value) {
	dshot_1.sendThrottle(throttle_value);
	dshot_2.sendThrottle(throttle_value);
	dshot_3.sendThrottle(throttle_value);
	dshot_4.sendThrottle(throttle_value);
}
