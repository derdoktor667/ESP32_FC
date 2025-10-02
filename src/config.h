#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Define IBUS RX Pin for Serial2 - YOU NEED TO ADJUST THIS TO YOUR SETUP
// Default RX pin for Serial2 on ESP32 is GPIO_NUM_16
const gpio_num_t IBUS_RX_PIN = GPIO_NUM_16;

// Define ESC pins - YOU NEED TO ADJUST THESE TO YOUR SETUP
const gpio_num_t ESC_PIN_1 = GPIO_NUM_27;
const gpio_num_t ESC_PIN_2 = GPIO_NUM_25;
const gpio_num_t ESC_PIN_3 = GPIO_NUM_26;
const gpio_num_t ESC_PIN_4 = GPIO_NUM_33;

// Constants for IBUS channels
const int IBUS_MIN_VALUE = 1000;
const int IBUS_MAX_VALUE = 2000;
const int IBUS_CHANNEL_THROTTLE = 0;
const int IBUS_CHANNEL_ROLL = 1;
const int IBUS_CHANNEL_PITCH = 2;
const int IBUS_CHANNEL_YAW = 3;
const int IBUS_CHANNEL_ARMING = 4;
const int IBUS_CHANNEL_FLIGHT_MODE = 5; // Assuming Channel 5 is the flight mode switch
const int IBUS_ARMING_THRESHOLD = 1500;

// Constants for DShot throttle range
const int DSHOT_MIN_THROTTLE = 48;
const int DSHOT_MAX_THROTTLE = 2047;

// Constants for PID integral limits
const float PID_INTEGRAL_LIMIT = 400.0;

// Constants for target angle ranges
const float TARGET_ANGLE_ROLL_PITCH = 30.0; // Degrees
const float TARGET_ANGLE_YAW_RATE = 90.0;   // Degrees/second (rate control)

// Constants for MPU6050 calibration
const int MPU_CALIBRATION_READINGS = 1000;
const float ACCEL_Z_GRAVITY = 1.0; // Assuming Z-axis is up and should read 1g

// Complementary filter gain
const float COMPLEMENTARY_FILTER_GAIN = 0.98; // Adjust as needed

// Serial print interval
const unsigned long PRINT_INTERVAL_MS = 100; // Print every 100 milliseconds

#endif
