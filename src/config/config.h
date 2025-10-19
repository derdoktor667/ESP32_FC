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
#include <ESP32_MPU6050.h>

// =================================================================================
// Hardware Configuration (do not change unless you modify the hardware)
// =================================================================================

// Define IBUS RX Pin for Serial2
// Default RX pin for Serial2 on ESP32 is GPIO_NUM_16
const gpio_num_t IBUS_RX_PIN = GPIO_NUM_16;

// Define ESC pins
const gpio_num_t ESC_PIN_FRONT_RIGHT = GPIO_NUM_27; // Front-Right motor
const gpio_num_t ESC_PIN_FRONT_LEFT = GPIO_NUM_25;  // Front-Left motor
const gpio_num_t ESC_PIN_REAR_RIGHT = GPIO_NUM_26;  // Rear-Right motor
const gpio_num_t ESC_PIN_REAR_LEFT = GPIO_NUM_33;   // Rear-Left motor

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

// Supported IMU Rotations
enum ImuRotation
{
    IMU_ROTATION_NONE,
    IMU_ROTATION_90_DEG_CW,
    IMU_ROTATION_180_DEG_CW,
    IMU_ROTATION_270_DEG_CW,
    IMU_ROTATION_FLIP, // 180 degree roll
    IMU_ROTATION_COUNT // Keep this last to count the number of rotations
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

// Default iBUS Channel Mappings (0-indexed)
static constexpr int IBUS_CHANNEL_THROTTLE = 1;
static constexpr int IBUS_CHANNEL_ROLL = 0;
static constexpr int IBUS_CHANNEL_PITCH = 2;
static constexpr int IBUS_CHANNEL_YAW = 3;
static constexpr int IBUS_CHANNEL_ARM_SWITCH = 4;
static constexpr int IBUS_CHANNEL_FAILSAFE_SWITCH = 5;
static constexpr int IBUS_CHANNEL_FLIGHT_MODE_SWITCH = 6;

// Structs for organizing FlightControllerSettings
struct PidAxisSettings
{
    int kp, ki, kd;
};

struct PidSettings
{
    PidAxisSettings roll{800, 1, 50};
    PidAxisSettings pitch{800, 1, 50};
    PidAxisSettings yaw{1500, 5, 100};
    float integralLimit = 400.0f;
};

struct RateSettings
{
    float maxAngleRollPitch = 30.0f;
    float maxRateYaw = 90.0f;
    float maxRateRollPitch = 90.0f;
};

struct CalibrationSettings
{
    int mpuCalibrationReadings = 1000;
    float accelZGravity = 1.0f;
};

struct FilterSettings
{
    float complementaryFilterTau = 0.995f;
    float gyroLpfCutoffFreq = 10.0f;
    float accelLpfCutoffFreq = 5.0f;
    uint8_t gyroLpfStages = 2;
    uint8_t accelLpfStages = 2;
    float filterSampleFreq = 250.0f;

    bool enableGyroNotchFilter = false;
    float gyroNotchFreq = 80.0f;
    float gyroNotchQ = 1.0f;
};

struct ReceiverChannelMapping
{
    int channel[NUM_FLIGHT_CONTROL_INPUTS] = {
        [THROTTLE] = IBUS_CHANNEL_THROTTLE,
        [ROLL] = IBUS_CHANNEL_ROLL,
        [PITCH] = IBUS_CHANNEL_PITCH,
        [YAW] = IBUS_CHANNEL_YAW,
        [ARM_SWITCH] = IBUS_CHANNEL_ARM_SWITCH,
        [FAILSAFE_SWITCH] = IBUS_CHANNEL_FAILSAFE_SWITCH,
        [FLIGHT_MODE_SWITCH] = IBUS_CHANNEL_FLIGHT_MODE_SWITCH
    };
};

struct ReceiverSettings
{
    ReceiverProtocol protocol = PROTOCOL_IBUS;
    ReceiverChannelMapping channelMapping;
    int minValue = 1000;
    int maxValue = 2000;
    int armingThreshold = 1500;
    int failsafeThreshold = 1500;
};

struct ImuSettings
{
    ImuProtocol protocol = IMU_MPU6050;
    LpfBandwidth lpfBandwidth = LPF_256HZ_N_0MS;
    ImuRotation rotation = IMU_ROTATION_180_DEG_CW;
};

struct MotorSettings
{
    float idleSpeedPercent = 4.0f;
    dshot_mode_t dshotMode = DSHOT600;
};

struct LoggingSettings
{
    unsigned long printIntervalMs = 40;
    bool enableLogging = false;
    bool enforceLoopTime = true;
};


struct FlightControllerSettings
{
    ImuSettings imu;
    ReceiverSettings receiver;
    PidSettings pid;
    RateSettings rates;
    CalibrationSettings calibration;
    FilterSettings filter;
    MotorSettings motor;
    LoggingSettings logging;
};

static constexpr int PID_SCALE_FACTOR = 1000;

static constexpr uint16_t MAX_UINT16_VALUE = 65535;
static constexpr uint8_t MAX_UINT8_VALUE = 255;

// General Constants

static constexpr int RECEIVER_CHANNEL_COUNT = 16;
static constexpr int NUM_MOTORS = 4;
static constexpr unsigned long CLI_REBOOT_DELAY_MS = 100;
static constexpr unsigned long IMU_INIT_FAIL_DELAY_MS = 10;
static constexpr unsigned long SERIAL_BAUD_RATE = 115200;
static constexpr int RX_MAP_PREFIX_LENGTH = 7;
static constexpr int RECEIVER_CHANNEL_OFFSET = 1;      // Offset for receiver channel indexing (e.g., 0-indexed array to 1-indexed channel)
static constexpr float SETPOINT_SCALING_FACTOR = 2.0f; // Factor used for scaling receiver input to setpoints
static constexpr bool INFINITE_LOOP_CONDITION = true;  // Used to halt execution on critical errors, preventing further processing.

static constexpr int FLIGHT_MODE_ACRO_THRESHOLD = 1200;
static constexpr int FLIGHT_MODE_ANGLE_THRESHOLD = 1800;

// DShot Motor Constants
static constexpr int DSHOT_MIN_THROTTLE = 48;   // Minimum DShot throttle value (e.g., for motor idle)
static constexpr int DSHOT_MAX_THROTTLE = 2047; // Maximum DShot throttle value

// Flight Loop Timing
static constexpr unsigned long TARGET_LOOP_TIME_US = 1000; // Target loop time in microseconds (1 kHz)

// Attitude Estimation
static constexpr size_t ATTITUDE_BUFFER_SIZE = 16; // Buffer size for attitude float to string conversion

// Firmware Version
static constexpr const char *FIRMWARE_VERSION = "0.2.6"; // Updated for release

// Simple compile-time string hash function (DJB2 variant, truncated to 15 bits)
constexpr uint16_t compileTimeHash15Bit(const char* str) {
    uint32_t hash = 5381;
    int c;
    while ((c = *str++)) {
        hash = ((hash << 5) + hash) + c; // hash * 33 + c
    }
    return static_cast<uint16_t>(hash & 0x7FFF); // Truncate to 15 bits (0 to 32767)
}

static constexpr uint16_t FIRMWARE_BUILD_ID = compileTimeHash15Bit(__DATE__ " " __TIME__); // Unique ID for each build

#endif