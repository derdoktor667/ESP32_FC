/*
 Name:		ESP32_ESC.ino
 Created:	20.03.2021 00:49:15
 Author:	derdoktor667
*/

#include <Arduino.h>

#include "fc_config.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "src/FlySkyIBUS/FlySkyIBUS.h"
#include "src/DShotRMT/DShotRMT.h"
#include "src/SystemState/SystemState.h"
#include "src/MPU6050/MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

// ...is Bluetooth enabled???
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

constexpr auto MOTOR_1 = GPIO_NUM_4;
constexpr auto MOTOR_2 = GPIO_NUM_0;
constexpr auto MOTOR_3 = GPIO_NUM_27;
constexpr auto MOTOR_4 = GPIO_NUM_26;

// ...hardware init
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

FlySkyIBUS ibus;
MPU6050 mpu;

DShotRMT dshot_1(MOTOR_1, RMT_CHANNEL_0);
DShotRMT dshot_2(MOTOR_2, RMT_CHANNEL_1);
DShotRMT dshot_3(MOTOR_3, RMT_CHANNEL_2);
DShotRMT dshot_4(MOTOR_4, RMT_CHANNEL_3);

volatile auto throttle_value = 0x30;

void setup() {
	// ...always start the onboard usb support
	USB_Serial.begin(USB_SERIAL_BAUD);

	// ...OLED
	// if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
	// USB_Serial.println("...OLED Display not found - STOP");
	// for (;;);
	// }

	// IBUS
	ibus.begin();

	// ...select the DSHOT Mode
	dshot_1.begin(DSHOT600);
	dshot_2.begin(DSHOT600);
	dshot_3.begin(DSHOT600);
	dshot_4.begin(DSHOT600);
}

void loop() {
	update_throttle_reading();

	dshot_1.send_dshot_value(throttle_value);
	dshot_2.send_dshot_value(throttle_value);
	dshot_3.send_dshot_value(throttle_value);
	dshot_4.send_dshot_value(throttle_value);

	USB_Serial.println(throttle_value);
}

//void readSerialThrottle() {
//	if (USB_Serial.available() > 0) {
//		auto throttle_input = (USB_Serial.readStringUntil('\n')).toInt();
//		throttle_value = throttle_input;
//	}
//}

void update_throttle_reading() {
	auto rc_readings_All = ibus.get_IBUS_Channels();
	throttle_value = rc_readings_All[THROTTLE];
}