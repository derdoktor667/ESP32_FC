//
// Name:		MotorDriver.cpp
// Created: 	19.07.2021 20:13:31
// Author:  	derdoktor667
//

#include "MotorDriver.h"

void init_motor(uint8_t motor_number, motor_type_t type) {
    motor_type[motor_number] = type;
};

motor_type_t get_motor_type(uint8_t motor_number) {
    return motor_type[motor_number];
};

void set_Signal_Rx_Type(rx_type_t _rx_type) {
    rx_type = _rx_type;
 };

rx_type_t get_Signal_Rx_Type() {
    return rx_type;
};

void set_MotorSignal(uint8_t motor_Nr, uint16_t signal) {
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
            if ((Serial.available() > 0) | (signal = 0)) {
                throttle_input = (Serial.readStringUntil('\n')).toInt();
                motor_throttle[motor_Nr] = throttle_input;
                }

            break;
    }

    switch (motor_type[motor_Nr]) {
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
