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

// Supported IMU Protocols
enum ImuProtocol
{
    IMU_MPU6050, // MPU6050 sensor
    // Add other IMU protocols here as needed
};

// Flight Control Inputs
enum FlightControlInput
{
    THROTTLE,
    ROLL,
    PITCH,
    YAW,
    ARM_SWITCH,
    FAILSAFE_SWITCH,
    FLIGHT_MODE_SWITCH,
    // Add other control inputs as needed
    NUM_FLIGHT_CONTROL_INPUTS // Keep this last to count the number of inputs
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

    // IMU Protocol Selection
    ImuProtocol imuProtocol = IMU_MPU6050;

    // Receiver Channel Mapping
    struct
    {
        int channel[NUM_FLIGHT_CONTROL_INPUTS];
    } channelMapping = {
        .channel = {
            [THROTTLE] = 1, // iBUS Channel 2 (0-indexed)
            [ROLL] = 0,     // iBUS Channel 1
            [PITCH] = 2,    // iBUS Channel 3
            [YAW] = 3,      // iBUS Channel 4
            [ARM_SWITCH] = 4, // iBUS Channel 5
            [FAILSAFE_SWITCH] = 5, // iBUS Channel 6
            [FLIGHT_MODE_SWITCH] = 6 // iBUS Channel 7
        }
    };

    // PID Controller Gains
    struct
    {
        int kp, ki, kd;
    } pidRoll{800, 1, 50},
        pidPitch{800, 1, 50},
        pidYaw{1500, 5, 100};

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
        float madgwickSampleFreq = 250.0f; // Hz
        float madgwickBeta = 0.1f;         // Madgwick filter gain
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

    // Motor Settings
    float motorIdleSpeedPercent = 4.0f; // Minimum throttle percentage for motors when armed
};

static constexpr int PID_SCALE_FACTOR = 1000;

// General Constants
static constexpr int RECEIVER_PROTOCOL_COUNT = 2;
static constexpr int IMU_PROTOCOL_COUNT = 1;
static constexpr int RECEIVER_CHANNEL_COUNT = 16;
static constexpr unsigned long CLI_REBOOT_DELAY_MS = 100;
static constexpr unsigned long IMU_INIT_FAIL_DELAY_MS = 10;
static constexpr unsigned long SERIAL_BAUD_RATE = 115200;

// Create a single, global instance of the settings
// extern FlightControllerSettings settings; // Now declared in settings.h

#endif