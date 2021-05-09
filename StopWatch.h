// Author:	derdoktor667
//

#pragma once

#include <Arduino.h>

extern HardwareSerial* USB_Serial;

class StopWatch {
	public:
		StopWatch(char* stopWatch_Name = "Looptime");
		inline ~StopWatch() {
			USB_Serial->print("Looptime ");
			USB_Serial->print(this->stopWatch_Name);
			USB_Serial->print(": ");
			USB_Serial->print((int(esp_timer_get_time() - this->start)));
			USB_Serial->println(" Microseconds");
		}

	private:
		char* stopWatch_Name;
		clock_t start;
};
