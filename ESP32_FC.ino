/*
 Name:		ESP32_ESC.ino
 Created:	20.03.2021 00:49:15
 Author:	derdoktor667
*/

#include <Arduino.h>
#include "common.h"
#include "DShotRMT.h"
#include <SSD1306.h>

// ...hardware init
HardwareSerial* USB_Serial = &Serial;
DShotRMT dshot(GPIO_NUM_2, RMT_CHANNEL_0);
SSD1306 Display(OLED_ADDRESS, SDA_PIN, SCL_PIN);

// the setup function runs once when you press reset or power the board
void setup() {
	USB_Serial->begin(USB_SERIAL_BAUD);
	USB_Serial->setTimeout(portTICK_PERIOD_MS);

	//
	Display.init();
	Display.flipScreenVertically();
	Display.clear();
	Display.display();

	// ...select the DSHOT Mode
	dshot.init(DSHOT300);
}

volatile auto x = 48;

// the loop function runs over and over again until power down or reset
void loop() {
	readSerialThrottle();
	dshot.sendThrottle(x);
	
	// ...lags about 3ms if enabled
	// updateDisplay();
}

void updateDisplay() {
	String dshotNameString = dshot.get_dshot_mode();

	Display.clear();
	Display.setTextAlignment(TEXT_ALIGN_LEFT);
	Display.drawString(0, 0, "DSHOT Mode:");
	Display.drawString(0, 12, "DSHOT CMD:");
	Display.drawString(0, 24, " ");
	Display.drawString(0, 36, "Looptime ms:");
	Display.setTextAlignment(TEXT_ALIGN_RIGHT);
	Display.drawString(128, 0, dshotNameString);
	Display.drawString(128, 12, String(x));
	Display.drawString(128, 24, String(" "));
	Display.drawString(128, 36, String(" "));
	Display.display();
}

void readSerialThrottle() {
	if (USB_Serial->available() > 0) {
		String str = USB_Serial->readStringUntil('\n');
		x = str.toInt();
	}
}