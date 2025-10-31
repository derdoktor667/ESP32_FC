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
const gpio_num_t RECEIVER_RX_PIN = GPIO_NUM_16;

// Define ESC pins
const gpio_num_t ESC_PIN_FRONT_RIGHT = GPIO_NUM_27; // Front-Right motor
const gpio_num_t ESC_PIN_FRONT_LEFT = GPIO_NUM_25;  // Front-Left motor
const gpio_num_t ESC_PIN_REAR_RIGHT = GPIO_NUM_26;  // Rear-Right motor
const gpio_num_t ESC_PIN_REAR_LEFT = GPIO_NUM_33;   // Rear-Left motor

// =================================================================================
// Flight Controller Settings (user-adjustable parameters)
// =================================================================================

// Supported Receiver Protocols
enum ReceiverProtocol : uint8_t
{
    PROTOCOL_IBUS,
    PROTOCOL_PPM,
    RECEIVER_PROTOCOL_COUNT // Keep this last to count the number of protocols
};

// Supported IMU Protocols
enum ImuProtocol : uint8_t
{
    IMU_MPU6050,
    IMU_PROTOCOL_COUNT // Keep this last to count the number of protocols
};

// Supported IMU Rotations
enum ImuRotation : uint8_t
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
    NUM_FLIGHT_CONTROL_INPUTS // Keep this last to count the number of inputs
};

// Flight Modes
enum FlightMode
{
    ACRO_MODE,
    ANGLE_MODE,
};

enum MotorPosition
{
    MOTOR_FRONT_RIGHT = 0,
    MOTOR_FRONT_LEFT = 1,
    MOTOR_REAR_RIGHT = 2,
    MOTOR_REAR_LEFT = 3,
    NUM_MOTORS_ENUM // Keep this last to count the number of motors
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
struct PidAxisGains
{
    int kp, ki, kd;
};

// PID Default Values
static constexpr int DEFAULT_PID_KP_ROLL_PITCH = 800;
static constexpr int DEFAULT_PID_KI_ROLL_PITCH = 1;
static constexpr int DEFAULT_PID_KD_ROLL_PITCH = 50;
static constexpr int DEFAULT_PID_KP_YAW = 1500;
static constexpr int DEFAULT_PID_KI_YAW = 5;
static constexpr int DEFAULT_PID_KD_YAW = 100;
static constexpr float DEFAULT_PID_INTEGRAL_LIMIT = 400.0f;

struct PidSettings
{
    PidAxisGains roll{DEFAULT_PID_KP_ROLL_PITCH, DEFAULT_PID_KI_ROLL_PITCH, DEFAULT_PID_KD_ROLL_PITCH};
    PidAxisGains pitch{DEFAULT_PID_KP_ROLL_PITCH, DEFAULT_PID_KI_ROLL_PITCH, DEFAULT_PID_KD_ROLL_PITCH};
    PidAxisGains yaw{DEFAULT_PID_KP_YAW, DEFAULT_PID_KI_YAW, DEFAULT_PID_KD_YAW};
    float integralLimit = DEFAULT_PID_INTEGRAL_LIMIT;
};

// Rate Default Values
static constexpr float DEFAULT_MAX_ANGLE_ROLL_PITCH = 30.0f;
static constexpr float DEFAULT_MAX_RATE_YAW = 90.0f;
static constexpr float DEFAULT_MAX_RATE_ROLL_PITCH = 90.0f;

struct FlightRateSettings
{
    float maxAngleRollPitch = DEFAULT_MAX_ANGLE_ROLL_PITCH;
    float maxRateYaw = DEFAULT_MAX_RATE_YAW;
    float maxRateRollPitch = DEFAULT_MAX_RATE_ROLL_PITCH;
};

// Calibration Default Values
static constexpr int DEFAULT_MPU_CALIBRATION_READINGS = 1000;
static constexpr float DEFAULT_ACCEL_Z_GRAVITY = 1.0f;

struct CalibrationSettings
{
    int mpuCalibrationReadings = DEFAULT_MPU_CALIBRATION_READINGS;
    float accelZGravity = DEFAULT_ACCEL_Z_GRAVITY;
};

// Filter Default Values
static constexpr float DEFAULT_COMPLEMENTARY_FILTER_TAU = 0.995f;
static constexpr float DEFAULT_GYRO_LPF_CUTOFF_FREQ = 1.0f; // Default gyroscope low-pass filter cutoff frequency in Hz. A lower value reduces noise but increases latency.
static constexpr float DEFAULT_ACCEL_LPF_CUTOFF_FREQ = 5.0f;
static constexpr uint8_t DEFAULT_GYRO_LPF_STAGES = 2;
static constexpr uint8_t DEFAULT_ACCEL_LPF_STAGES = 2;
static constexpr float DEFAULT_FILTER_SAMPLE_FREQ = 250.0f;
static constexpr bool DEFAULT_ENABLE_GYRO_NOTCH_FILTER = false;
static constexpr float DEFAULT_GYRO_NOTCH_FREQ = 80.0f;
static constexpr float DEFAULT_GYRO_NOTCH_Q = 1.0f;

struct FilterSettings
{
    float complementaryFilterTau = DEFAULT_COMPLEMENTARY_FILTER_TAU;
    float gyroLpfCutoffFreq = DEFAULT_GYRO_LPF_CUTOFF_FREQ;
    float accelLpfCutoffFreq = DEFAULT_ACCEL_LPF_CUTOFF_FREQ;
    uint8_t gyroLpfStages = DEFAULT_GYRO_LPF_STAGES;
    uint8_t accelLpfStages = DEFAULT_ACCEL_LPF_STAGES;
    float filterSampleFreq = DEFAULT_FILTER_SAMPLE_FREQ;

    bool enableGyroNotchFilter = DEFAULT_ENABLE_GYRO_NOTCH_FILTER;
    float gyroNotchFreq = DEFAULT_GYRO_NOTCH_FREQ;
    float gyroNotchQ = DEFAULT_GYRO_NOTCH_Q;
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

// Receiver Default Values
static constexpr int DEFAULT_RX_MIN_VALUE = 1000;
static constexpr int DEFAULT_RX_MAX_VALUE = 2000;
static constexpr int DEFAULT_RX_ARMING_THRESHOLD = 1500;
static constexpr int DEFAULT_RX_FAILSAFE_THRESHOLD = 1500;

struct ReceiverSettings
{
    ReceiverProtocol protocol = PROTOCOL_IBUS;
    ReceiverChannelMapping channelMapping;
    int minValue = DEFAULT_RX_MIN_VALUE;
    int maxValue = DEFAULT_RX_MAX_VALUE;
    int armingThreshold = DEFAULT_RX_ARMING_THRESHOLD;
    int failsafeThreshold = DEFAULT_RX_FAILSAFE_THRESHOLD;
};

struct ImuSettings
{
    ImuProtocol protocol = IMU_MPU6050;
    LpfBandwidth lpfBandwidth = LPF_42HZ_N_5MS;
    ImuRotation rotation = IMU_ROTATION_180_DEG_CW;
};

// Motor Default Values
static constexpr float DEFAULT_MOTOR_IDLE_SPEED_PERCENT = 4.0f;
static constexpr dshot_mode_t DEFAULT_DSHOT_MODE = DSHOT600;

struct MotorSettings
{
    float idleSpeedPercent = DEFAULT_MOTOR_IDLE_SPEED_PERCENT;
    dshot_mode_t dshotMode = DEFAULT_DSHOT_MODE;
};

// Logging Default Values
static constexpr unsigned long DEFAULT_PRINT_INTERVAL_MS = 40;
static constexpr bool DEFAULT_ENABLE_LOGGING = false;
static constexpr bool DEFAULT_ENFORCE_LOOP_TIME = true;
static constexpr bool DEFAULT_ENABLE_BENCH_RUN_MODE = false;
static constexpr const char* DEFAULT_TEST_STRING = "default_test_string";

struct LoggingSettings
{
    unsigned long printIntervalMs = DEFAULT_PRINT_INTERVAL_MS;
    bool enableLogging = DEFAULT_ENABLE_LOGGING;
    bool enforceLoopTime = DEFAULT_ENFORCE_LOOP_TIME;
    bool enableBenchRunMode = DEFAULT_ENABLE_BENCH_RUN_MODE;
    bool enableDebugLogging = false;
    bool inCliMode = false; // Add this line
    String testString = DEFAULT_TEST_STRING; // Added for MSP string testing
};


struct FlightControllerSettings
{
    ImuSettings imu;
    ReceiverSettings receiver;
    PidSettings pid;
    FlightRateSettings flightRates;
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

// Flight Mode Thresholds
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
static constexpr const char *FIRMWARE_VERSION = "0.2.6";

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