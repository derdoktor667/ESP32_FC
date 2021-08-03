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

volatile system_state_t SystemState = ERROR;
