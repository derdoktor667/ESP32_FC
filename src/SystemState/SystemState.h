//
// Name:		SystemState.h
// Created: 	20.03.2021 00:49:15
// Author:  	derdoktor667
//

#pragma once

typedef enum system_state {
    RUNNING,
	ERROR,
	CONFIG,
} system_state_t;

typedef enum rx_type {
	NO_RX,
	PPM,
	IBUS,
	SBUS,
} rx_type_t;

typedef enum rx_mode {
	MODE_1 = 1,
	MODE_2,
	MODE_3,
	MODE_4,
} rx_mode_t;
		
typedef enum motor_type {
	OFF,
	PWM,
	ONESHOT,
	DSHOT,
 } motor_type_t;

typedef enum flight_mode {
	LEVEL,
	STUNT,
	ACRO,
} flight_mode_t;
