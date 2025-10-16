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

#include "src/config/config.h"

// Declare the global settings instance to be accessible across the project
extern FlightControllerSettings settings;

// NVS Keys for settings persistence
// These keys are used to store and retrieve settings from the ESP32's flash memory.
namespace NVSKeys {
    static constexpr const char* PID_ROLL_KP = "pid.r.kp";
    static constexpr const char* PID_ROLL_KI = "pid.r.ki";
    static constexpr const char* PID_ROLL_KD = "pid.r.kd";
    static constexpr const char* PID_PITCH_KP = "pid.p.kp";
    static constexpr const char* PID_PITCH_KI = "pid.p.ki";
    static constexpr const char* PID_PITCH_KD = "pid.p.kd";
    static constexpr const char* PID_YAW_KP = "pid.y.kp";
    static constexpr const char* PID_YAW_KI = "pid.y.ki";
    static constexpr const char* PID_YAW_KD = "pid.y.kd";
    static constexpr const char* PID_INTEGRAL_LIMIT = "pid.lim";

    static constexpr const char* RATES_MAX_ANGLE_ROLL_PITCH = "rate.angle";
    static constexpr const char* RATES_MAX_RATE_YAW = "rate.y";
    static constexpr const char* RATES_MAX_RATE_ROLL_PITCH = "rate.acro_rp";

    static constexpr const char* FILTER_MADGWICK_SAMPLE_FREQ = "filter.madgwick.sample_freq";
    static constexpr const char* FILTER_MADGWICK_BETA = "filter.madgwick.beta";

    static constexpr const char* RX_PROTOCOL = "rx.proto";
    static constexpr const char* RX_MIN_VALUE = "rx.min";
    static constexpr const char* RX_MAX_VALUE = "rx.max";
    static constexpr const char* RX_ARMING_THRESHOLD = "rx.arming";
    static constexpr const char* RX_FAILSAFE_THRESHOLD = "rx.failsafe";
    static constexpr const char* RX_CHANNEL_MAP_PREFIX = "rx.map."; // Prefix for channel mapping keys

    static constexpr const char* IMU_PROTOCOL = "imu.proto";
    static constexpr const char* IMU_LPF_BANDWIDTH = "imu.lpf";

    static constexpr const char* MOTOR_IDLE_SPEED = "motor.idle";
    static constexpr const char* DSHOT_MODE_VAL = "dshot_mode_val";
    static constexpr const char* ENFORCE_LOOP_TIME = "enforce.loop";
}

// Functions to manage persistent storage
void saveSettings();
void loadSettings();

#endif
