// settings.h
//
// This file declares the global FlightControllerSettings instance and functions
// for managing persistent storage (Non-Volatile Storage - NVS) of these settings.
// It also defines the NVS keys used for storing individual settings.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#ifndef SETTINGS_H
#define SETTINGS_H

#include <functional> // Required for std::function
#include "src/config/config.h"

// Declare the global settings instance to be accessible across the project
extern FlightControllerSettings settings;

// NVS Keys for settings persistence
// These keys are used to store and retrieve settings from the ESP32's flash memory.
namespace NVSKeys
{
    static constexpr const char *PID_ROLL_KP = "pid.roll.kp";
    static constexpr const char *PID_ROLL_KI = "pid.roll.ki";
    static constexpr const char *PID_ROLL_KD = "pid.roll.kd";
    static constexpr const char *PID_PITCH_KP = "pid.pitch.kp";
    static constexpr const char *PID_PITCH_KI = "pid.pitch.ki";
    static constexpr const char *PID_PITCH_KD = "pid.pitch.kd";
    static constexpr const char *PID_YAW_KP = "pid.yaw.kp";
    static constexpr const char *PID_YAW_KI = "pid.yaw.ki";
    static constexpr const char *PID_YAW_KD = "pid.yaw.kd";
    static constexpr const char *PID_INTEGRAL_LIMIT = "pid.int_limit";

    static constexpr const char *RATES_MAX_ANGLE_ROLL_PITCH = "rate.angle";
    static constexpr const char *RATES_MAX_RATE_YAW = "rate.y";
    static constexpr const char *RATES_MAX_RATE_ROLL_PITCH = "rate.acro_rp";

    static constexpr const char *FILTER_COMPLEMENTARY_TAU = "filter.comp_tau";
    static constexpr const char *FILTER_GYRO_LPF_CUTOFF_FREQ = "gyro.lpf_cfreq";
    static constexpr const char *FILTER_ACCEL_LPF_CUTOFF_FREQ = "accel.lpf_cfreq";
    static constexpr const char *FILTER_GYRO_LPF_STAGES = "gyro.lpf_stages";
    static constexpr const char *FILTER_ACCEL_LPF_STAGES = "accel.lpf_stg";
    static constexpr const char *FILTER_SAMPLE_FREQ = "filter.s_freq";
    static constexpr const char *FILTER_GYRO_NOTCH_ENABLE = "gyro.notch.en";
    static constexpr const char *FILTER_GYRO_NOTCH_FREQ = "gyro.notch.freq";
    static constexpr const char *FILTER_GYRO_NOTCH_Q = "gyro.notch.q";

    static constexpr const char *RX_PROTOCOL = "rx.proto";
    static constexpr const char *RX_MIN_VALUE = "rx.min";
    static constexpr const char *RX_MAX_VALUE = "rx.max";
    static constexpr const char *RX_ARMING_THRESHOLD = "rx.arming";
    static constexpr const char *RX_FAILSAFE_THRESHOLD = "rx.failsafe";
    static constexpr const char *RX_CHANNEL_MAP_PREFIX = "rx.map."; // Prefix for channel mapping keys

    static constexpr const char *IMU_PROTOCOL = "imu.protocol";
    static constexpr const char *IMU_LPF_BANDWIDTH = "imu.lpf";
    static constexpr const char *IMU_ROTATION = "imu.rotation";

    static constexpr const char *MOTOR_IDLE_SPEED = "motor.idle_spd";
    static constexpr const char *DSHOT_MODE_VAL = "motor.dshot_m";
    static constexpr const char *ENFORCE_LOOP_TIME = "log.enforce_l";

    // Calibration Settings
    static constexpr const char *CALIBRATION_MPU_READINGS = "cal.mpu_read";
    static constexpr const char *CALIBRATION_ACCEL_Z_GRAVITY = "cal.accel_z_g";

    // Logging Settings
    static constexpr const char *LOGGING_PRINT_INTERVAL_MS = "log.print_int";
    static constexpr const char *LOGGING_ENABLE = "log.enable";
    static constexpr const char *BENCH_RUN_MODE_ENABLE = "bench.run.en";
    static constexpr const char *LOGGING_DEBUG_ENABLE = "log.debug.en"; // Add this line
    static constexpr const char *LOGGING_TEST_STRING = "log.test_str";
    static constexpr const char *FIRMWARE_BUILD_ID = "fw.build.id";
}

// Functions to manage persistent storage
void saveSettings(std::function<void(const String&)> sendDebugMessage = [](const String&){});
void loadSettings();

#endif
