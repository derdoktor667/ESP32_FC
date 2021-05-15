#include "StopWatch.h"

using namespace std;

StopWatch::StopWatch(char* stopWatch_Name) {
	this->stopWatch_Name = stopWatch_Name;
	this->start = esp_timer_get_time();
}
