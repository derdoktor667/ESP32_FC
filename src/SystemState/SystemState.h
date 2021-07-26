//
// Name:		SystemState.h
// Created: 	20.03.2021 00:49:15
// Author:  	derdoktor667
//

#pragma once

typedef enum system_state_e {
	RUNNING,
	ERROR,
	CONFIG,
	USB_MODE
} system_state_t;

typedef enum rx_mode_e {
	MODE_1 = 1,
	MODE_2,
	MODE_3,
	MODE_4,
} rx_mode_t;
		
typedef enum flight_mode_e {
	LEVEL,
	STUNT,
	ACRO,
} flight_mode_t;

extern volatile system_state_t SystemState = ERROR;
