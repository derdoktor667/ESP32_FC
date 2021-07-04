#pragma once

#include <Arduino.h>
#include "fc_config.h"

firmware_info_t firmware_info;

firmware_info_t* get_Firmware_Info() {
	return &firmware_info;
}
