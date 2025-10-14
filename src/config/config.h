// config.h
//
// This file contains global configuration settings, hardware definitions,
// and default values for the ESP32 Flight Controller project.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <DShotRMT.h>

// =================================================================================
// Hardware Configuration (do not change unless you modify the hardware)
// =================================================================================

// Define IBUS RX Pin for Serial2
// Default RX pin for Serial2 on ESP32 is GPIO_NUM_16
const gpio_num_t IBUS_RX_PIN = GPIO_NUM_16;

// Define ESC pins
const gpio_num_t ESC_PIN_FRONT_RIGHT = GPIO_NUM_27; // Front-Right motor
const gpio_num_t ESC_PIN_FRONT_LEFT = GPIO_NUM_25;  // Front-Left motor
const gpio_num_t ESC_PIN_REAR_RIGHT = GPIO_NUM_26; // Rear-Right motor
const gpio_num_t ESC_PIN_REAR_LEFT = GPIO_NUM_33;  // Rear-Left motor


// =================================================================================
// Flight Controller Settings (user-adjustable parameters)
// =================================================================================

// Supported Receiver Protocols
enum ReceiverProtocol
{
    PROTOCOL_IBUS,
    PROTOCOL_PPM,
    // PROTOCOL_SBUS, // Future implementation
    RECEIVER_PROTOCOL_COUNT // Keep this last to count the number of protocols
};

// Supported IMU Protocols
enum ImuProtocol
{
    IMU_MPU6050,
    // Add other IMU protocols here as needed
    IMU_PROTOCOL_COUNT // Keep this last to count the number of protocols
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

// Default values for FlightControllerSettings
static constexpr int DEFAULT_PID_ROLL_KP = 800;
static constexpr int DEFAULT_PID_ROLL_KI = 1;
static constexpr int DEFAULT_PID_ROLL_KD = 50;
static constexpr int DEFAULT_PID_PITCH_KP = 800;
static constexpr int DEFAULT_PID_PITCH_KI = 1;
static constexpr int DEFAULT_PID_PITCH_KD = 50;
static constexpr int DEFAULT_PID_YAW_KP = 1500;
static constexpr int DEFAULT_PID_YAW_KI = 5;
static constexpr int DEFAULT_PID_YAW_KD = 100;
static constexpr float DEFAULT_PID_INTEGRAL_LIMIT = 400.0f;

static constexpr float DEFAULT_MAX_ANGLE_ROLL_PITCH = 30.0f;
static constexpr float DEFAULT_MAX_RATE_YAW = 90.0f;
static constexpr float DEFAULT_MAX_RATE_ROLL_PITCH = 90.0f;

static constexpr int DEFAULT_MPU_CALIBRATION_READINGS = 1000;
static constexpr float DEFAULT_ACCEL_Z_GRAVITY = 1.0f;

static constexpr float DEFAULT_MADGWICK_SAMPLE_FREQ = 250.0f;
static constexpr float DEFAULT_MADGWICK_BETA = 0.1f;

static constexpr int DEFAULT_IBUS_MIN_VALUE = 1000;
static constexpr int DEFAULT_IBUS_MAX_VALUE = 2000;
static constexpr int DEFAULT_ARMING_THRESHOLD = 1500;
static constexpr int DEFAULT_FAILSAFE_THRESHOLD = 1500;

static constexpr unsigned long DEFAULT_PRINT_INTERVAL_MS = 40;

static constexpr float DEFAULT_MOTOR_IDLE_SPEED_PERCENT = 4.0f;
static constexpr dshot_mode_t DEFAULT_DSHOT_MODE = DSHOT600;

// iBUS Channel Mappings (0-indexed)
static constexpr int IBUS_CHANNEL_THROTTLE = 1;
static constexpr int IBUS_CHANNEL_ROLL = 0;
static constexpr int IBUS_CHANNEL_PITCH = 2;
static constexpr int IBUS_CHANNEL_YAW = 3;
static constexpr int IBUS_CHANNEL_ARM_SWITCH = 4;
static constexpr int IBUS_CHANNEL_FAILSAFE_SWITCH = 5;
static constexpr int IBUS_CHANNEL_FLIGHT_MODE_SWITCH = 6;

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
            [THROTTLE] = IBUS_CHANNEL_THROTTLE,
            [ROLL] = IBUS_CHANNEL_ROLL,
            [PITCH] = IBUS_CHANNEL_PITCH,
            [YAW] = IBUS_CHANNEL_YAW,
            [ARM_SWITCH] = IBUS_CHANNEL_ARM_SWITCH,
            [FAILSAFE_SWITCH] = IBUS_CHANNEL_FAILSAFE_SWITCH,
            [FLIGHT_MODE_SWITCH] = IBUS_CHANNEL_FLIGHT_MODE_SWITCH
        }
    };

    // PID Controller Gains
    struct
    {
        int kp, ki, kd;
    } pidRoll{DEFAULT_PID_ROLL_KP, DEFAULT_PID_ROLL_KI, DEFAULT_PID_ROLL_KD},
        pidPitch{DEFAULT_PID_PITCH_KP, DEFAULT_PID_PITCH_KI, DEFAULT_PID_PITCH_KD},
        pidYaw{DEFAULT_PID_YAW_KP, DEFAULT_PID_YAW_KI, DEFAULT_PID_YAW_KD};

    // PID Integral Wind-up Limit
    float pidIntegralLimit = DEFAULT_PID_INTEGRAL_LIMIT;

    // Target angle and rate limits
    struct
    {
        float maxAngleRollPitch = DEFAULT_MAX_ANGLE_ROLL_PITCH;
        float maxRateYaw = DEFAULT_MAX_RATE_YAW;
        float maxRateRollPitch = DEFAULT_MAX_RATE_ROLL_PITCH;
    } rates;

    // MPU6050 Calibration
    struct
    {
        int mpuCalibrationReadings = DEFAULT_MPU_CALIBRATION_READINGS;
        float accelZGravity = DEFAULT_ACCEL_Z_GRAVITY;
    } calibration;

    // Attitude Estimation
    struct
    {
        float madgwickSampleFreq = DEFAULT_MADGWICK_SAMPLE_FREQ;
        float madgwickBeta = DEFAULT_MADGWICK_BETA;
    } filter;

    // Receiver Settings
    struct
    {
        int ibusMinValue = DEFAULT_IBUS_MIN_VALUE;
        int ibusMaxValue = DEFAULT_IBUS_MAX_VALUE;
        int armingThreshold = DEFAULT_ARMING_THRESHOLD;
        int failsafeThreshold = DEFAULT_FAILSAFE_THRESHOLD;
    } receiver;

    // Serial Logging
    unsigned long printIntervalMs = DEFAULT_PRINT_INTERVAL_MS;
    bool enableLogging = false; // Global flag to enable/disable all logging output

    // Motor Settings
    float motorIdleSpeedPercent = DEFAULT_MOTOR_IDLE_SPEED_PERCENT;
    dshot_mode_t dshotMode = DEFAULT_DSHOT_MODE;
};

static constexpr int PID_SCALE_FACTOR = 1000;

// General Constants

static constexpr int RECEIVER_CHANNEL_COUNT = 16;
static constexpr int NUM_MOTORS = 4;
static constexpr unsigned long CLI_REBOOT_DELAY_MS = 100;
static constexpr unsigned long IMU_INIT_FAIL_DELAY_MS = 10;
static constexpr unsigned long SERIAL_BAUD_RATE = 115200;
static constexpr int RX_MAP_PREFIX_LENGTH = 7;
static constexpr bool INFINITE_LOOP_CONDITION = true; // Used to halt execution on critical errors, preventing further processing.

static constexpr int FLIGHT_MODE_ACRO_THRESHOLD = 1200;
static constexpr int FLIGHT_MODE_ANGLE_THRESHOLD = 1800;

// DShot Motor Constants
static constexpr int DSHOT_MIN_THROTTLE = 48;  // Minimum DShot throttle value (e.g., for motor idle)
static constexpr int DSHOT_MAX_THROTTLE = 2047; // Maximum DShot throttle value

// Firmware Version
static constexpr const char* FIRMWARE_VERSION = "0.2.5"; // Updated for release

#endif