#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =================================================================================
// Hardware Configuration (do not change unless you modify the hardware)
// =================================================================================

// Define IBUS RX Pin for Serial2
// Default RX pin for Serial2 on ESP32 is GPIO_NUM_16
const gpio_num_t IBUS_RX_PIN = GPIO_NUM_16;

// Define ESC pins
const gpio_num_t ESC_PIN_1 = GPIO_NUM_27; // Front-Right
const gpio_num_t ESC_PIN_2 = GPIO_NUM_25; // Front-Left
const gpio_num_t ESC_PIN_3 = GPIO_NUM_26; // Rear-Right
const gpio_num_t ESC_PIN_4 = GPIO_NUM_33; // Rear-Left


// =================================================================================
// Flight Controller Settings (user-adjustable parameters)
// =================================================================================

// Supported Receiver Protocols
enum ReceiverProtocol
{
    PROTOCOL_IBUS,
    PROTOCOL_PPM,  // Pulse-Position Modulation
    // PROTOCOL_SBUS, // Future implementation
};

// Flight Modes
enum FlightMode
{
  ACRO_MODE,
  ANGLE_MODE,
};

struct FlightControllerSettings
{
    // Receiver Protocol Selection
    ReceiverProtocol receiverProtocol = PROTOCOL_IBUS;

    // PID Controller Gains
    struct
    {
        float kp, ki, kd;
    } pidRoll{0.8, 0.001, 0.05},
        pidPitch{0.8, 0.001, 0.05},
        pidYaw{1.5, 0.005, 0.1};

    // PID Integral Wind-up Limit
    float pidIntegralLimit = 400.0;

    // Target angle and rate limits
    struct
    {
        float maxAngleRollPitch = 30.0; // Degrees for ANGLE_MODE
        float maxRateYaw = 90.0;        // Degrees/second for yaw axis
        float maxRateRollPitch = 90.0;  // Degrees/second for ACRO_MODE
    } rates;

    // MPU6050 Calibration
    struct
    {
        int mpuCalibrationReadings = 1000;
        float accelZGravity = 1.0; // Assuming Z-axis is up and should read 1g
    } calibration;

    // Attitude Estimation
    struct
    {
        float complementaryFilterGain = 0.98; // 98% gyro, 2% accelerometer
    } filter;

    // Receiver Settings
    struct
    {
        int ibusMinValue = 1000;
        int ibusMaxValue = 2000;
        int armingThreshold = 1500;
        int failsafeThreshold = 1500;
    } receiver;

    // Serial Logging
    unsigned long printIntervalMs = 100; // Print every 100 milliseconds
};

// Create a single, global instance of the settings
// extern FlightControllerSettings settings; // Now declared in settings.h

#endif