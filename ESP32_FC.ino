/*
 Name:		ESP32_ESC.ino
 Created:	20.03.2021 00:49:15
 Author:	derdoktor667
*/

#include <Arduino.h>

#include "common.h"
#include "DShotRMT.h"
#include "I2C.h"
#include "StopWatch.h"

constexpr auto MOTOR_1 = GPIO_NUM_4;
constexpr auto MOTOR_2 = GPIO_NUM_0;
constexpr auto MOTOR_3 = GPIO_NUM_27;
constexpr auto MOTOR_4 = GPIO_NUM_26;

// ...hardware init
DShotRMT dshot_1(MOTOR_1, RMT_CHANNEL_0);
DShotRMT dshot_2(MOTOR_2, RMT_CHANNEL_1);
DShotRMT dshot_3(MOTOR_3, RMT_CHANNEL_2);
DShotRMT dshot_4(MOTOR_4, RMT_CHANNEL_3);

volatile auto throttle_value = 48;

// the setup function runs once when you press reset or power the board
void setup() {
	USB_Serial->begin(USB_SERIAL_BAUD);

	// ...select the DSHOT Mode
	dshot_1.init(DSHOT600);
	dshot_2.init(DSHOT600);
	dshot_3.init(DSHOT600);
	dshot_4.init(DSHOT600);

	vTaskDelay(1000);
}


// the loop function runs over and over again until power down or reset
void loop() {
		readSerialThrottle();
		dshot_1.sendThrottle(throttle_value);
		dshot_2.sendThrottle(throttle_value);
		dshot_3.sendThrottle(throttle_value);
		dshot_4.sendThrottle(throttle_value);
}

void readSerialThrottle() {
	if (USB_Serial->available() > 0) {
		long throttle_input = (USB_Serial->readStringUntil('\n')).toInt();
		throttle_value = throttle_input;
	}
}
