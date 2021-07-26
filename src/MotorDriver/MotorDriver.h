//
// Name:		MotorDriver.h
// Created: 	19.07.2021 20:13:31
// Author:  	derdoktor667
//

#pragma once

#include <Arduino.h>
#include "../SystemState/SystemState.h"

typedef enum rx_type_e {
	NO_RX,
	PPM,
	IBUS,
	SBUS,
	TESTMODE_USB
} rx_type_t;

typedef enum motor_type_e {
	OFF,
	PWM,
	ONESHOT,
	DSHOT
 } motor_type_t;

// ...just to keep things going set MotoCount *fixed*
constexpr auto MOTOR_COUNT = 4;

extern volatile uint16_t motor_throttle[MOTOR_COUNT];
extern volatile motor_type_t motor_type[MOTOR_COUNT];
extern volatile rx_type_t rx_type = NO_RX;

void set_motor_type(uint8_t _motor_number, motor_type_t _type);
motor_type_t get_motor_type(uint8_t _motor_number);

void set_Signal_Rx_Type(rx_type_t _rx_type);
rx_type_t get_Signal_Rx_Type();

void set_motor_signal(uint8_t _motor_Nr, uint16_t _signal = 0);
