//
// Name:		MotorDriver.cpp
// Created: 	19.07.2021 20:13:31
// Author:  	derdoktor667
//

#include "MotorDriver.h"

void set_motor_type(uint8_t _motor_number, motor_type_t _type) {
    motor_type[_motor_number] = _type;
};

motor_type_t get_motor_type(uint8_t _motor_number) {
    return motor_type[_motor_number];
};

void set_Signal_Rx_Type(rx_type_t _rx_type) {
    rx_type = _rx_type;
 };

rx_type_t get_Signal_Rx_Type() {
    return rx_type;
};

void set_motor_signal(uint8_t _motor_Nr, uint16_t signal) {
    auto throttle_input = 0;

    if (Serial.available() > 0) {
        SystemState = USB_MODE;
    }

    switch (SystemState) {
        case RUNNING:
            break;

        case ERROR:
            break;

        case CONFIG:
            break;

        case USB_MODE:
            if ((Serial.available() > 0) || (signal == 0)) {
                throttle_input = (Serial.readStringUntil('\n')).toInt();
                motor_throttle[_motor_Nr] = throttle_input;
                }

            break;
    }

    switch (motor_type[_motor_Nr]) {
        case OFF:
            break;

        case PWM:
            break;

        case ONESHOT:
            break;

        case DSHOT:
            break;
    }

};
