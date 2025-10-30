// CommunicationManager.cpp
//
// This file implements the CommunicationManager class, which handles all serial
// communication (CLI/API) and logging for the ESP32 Flight Controller. It
// manages different operating modes (FLIGHT, CLI, API) and processes incoming
// commands and outgoing flight status data.
//
// Author: Wastl Kraus
// Date: 14.10.2025
// License: MIT

#include "CommunicationManager.h"
#include "flight_controller.h"
#include "../config/config.h"
#include "../config/settings.h"
#include <Arduino.h>

// --- Refactoring: Settings Registry ---

// --- Helper Functions for String Conversion ---
String CommunicationManager::_getReceiverProtocolName(ReceiverProtocol protocol) const
{
    if (protocol == PROTOCOL_IBUS)
        return "IBUS";
    if (protocol == PROTOCOL_PPM)
        return "PPM";
    return "UNKNOWN";
}

String CommunicationManager::_getImuProtocolName(ImuProtocol protocol) const
{
    if (protocol == IMU_MPU6050)
        return "MPU6050";
    return "UNKNOWN";
}

String CommunicationManager::_getLpfBandwidthName(LpfBandwidth bandwidth) const
{
    switch (bandwidth)
    {
    case LPF_256HZ_N_0MS:
        return "LPF_256HZ_N_0MS";
    case LPF_188HZ_N_2MS:
        return "LPF_188HZ_N_2MS";
    case LPF_98HZ_N_3MS:
        return "LPF_98HZ_N_3MS";
    case LPF_42HZ_N_5MS:
        return "LPF_42HZ_N_5MS";
    case LPF_20HZ_N_10MS:
        return "LPF_20HZ_N_10MS";
    case LPF_10HZ_N_13MS:
        return "LPF_10HZ_N_13MS";
    case LPF_5HZ_N_18MS:
        return "LPF_5HZ_N_18MS";
    default:
        return "UNKNOWN";
    }
}

String CommunicationManager::_getFlightControlInputName(FlightControlInput input) const
{
    switch (input)
    {
    case THROTTLE:
        return "THROTTLE";
    case ROLL:
        return "ROLL";
    case PITCH:
        return "PITCH";
    case YAW:
        return "YAW";
    case ARM_SWITCH:
        return "ARM_SWITCH";
    case FAILSAFE_SWITCH:
        return "FAILSAFE_SWITCH";
    case FLIGHT_MODE_SWITCH:
        return "FLIGHT_MODE_SWITCH";
    default:
        return "UNKNOWN";
    }
}

String CommunicationManager::_getDShotModeName(dshot_mode_t mode) const
{
    if (mode == DSHOT_OFF)
        return "DSHOT_OFF";
    if (mode == DSHOT150)
        return "DSHOT150";
    if (mode == DSHOT300)
        return "DSHOT300";
    if (mode == DSHOT600)
        return "DSHOT600";
    if (mode == DSHOT1200)
        return "DSHOT1200";
    return "UNKNOWN";
}

String CommunicationManager::_getImuRotationName(ImuRotation rotation) const
{
    switch (rotation)
    {
    case IMU_ROTATION_NONE:
        return "NONE";
    case IMU_ROTATION_90_DEG_CW:
        return "90_CW";
    case IMU_ROTATION_180_DEG_CW:
        return "180_CW";
    case IMU_ROTATION_270_DEG_CW:
        return "270_CW";
    case IMU_ROTATION_FLIP:
        return "FLIP";
    default:
        return "UNKNOWN";
    }
}

String CommunicationManager::_getBoolName(bool value) const
{
    return value ? "true" : "false";
}

String CommunicationManager::_getUint8Name(uint8_t value) const
{
    return String(value);
}

String CommunicationManager::_getULongName(unsigned long value) const
{
    return String(value);
}

String CommunicationManager::_convertPayloadToString(const uint8_t* payload, uint16_t size) const
{
    char buffer[size + 1];
    memcpy(buffer, payload, size);
    buffer[size] = '\0';
    return String(buffer);
}

uint16_t CommunicationManager::_serializeSettingToMspPayload(const Setting *setting, uint8_t *buffer) const
{
    uint16_t offset = 0;

    // Copy setting name (null-terminated string)
    strcpy((char *)(buffer + offset), setting->name);
    offset += strlen(setting->name) + 1; // +1 for null terminator

    // Copy setting type (1 byte)
    buffer[offset++] = (uint8_t)setting->type;

    // Copy setting value based on type
    switch (setting->type)
    {
    case SettingType::FLOAT:
    {
        float value = *(float *)setting->value / setting->scaleFactor;
        memcpy(buffer + offset, &value, sizeof(float));
        offset += sizeof(float);
        break;
    }
    case SettingType::INT:
    {
        int value = *(int *)setting->value;
        if (setting->scaleFactor == PID_SCALE_FACTOR) {
            value = value / PID_DISPLAY_SCALE_FACTOR;
        }
        memcpy(buffer + offset, &value, sizeof(int));
        offset += sizeof(int);
        break;
    }
    case SettingType::UINT8:
    {
        uint8_t value = *(uint8_t *)setting->value;
        memcpy(buffer + offset, &value, sizeof(uint8_t));
        offset += sizeof(uint8_t);
        break;
    }
    case SettingType::UINT16:
    {
        uint16_t value = *(uint16_t *)setting->value;
        memcpy(buffer + offset, &value, sizeof(uint16_t));
        offset += sizeof(uint16_t);
        break;
    }
    case SettingType::ULONG:
    {
        unsigned long value = *(unsigned long *)setting->value;
        memcpy(buffer + offset, &value, sizeof(unsigned long));
        offset += sizeof(unsigned long);
        break;
    }
    case SettingType::BOOL:
    {
        bool value = *(bool *)setting->value;
        buffer[offset++] = (uint8_t)value;
        break;
    }
    case SettingType::ENUM_IBUS_PROTOCOL:
    {
        ReceiverProtocol value = *(ReceiverProtocol *)setting->value;
        memcpy(buffer + offset, &value, sizeof(ReceiverProtocol));
        offset += sizeof(ReceiverProtocol);
        break;
    }
    case SettingType::ENUM_IMU_PROTOCOL:
    {
        ImuProtocol value = *(ImuProtocol *)setting->value;
        memcpy(buffer + offset, &value, sizeof(ImuProtocol));
        offset += sizeof(ImuProtocol);
        break;
    }
    case SettingType::ENUM_LPF_BANDWIDTH:
    {
        LpfBandwidth value = *(LpfBandwidth *)setting->value;
        memcpy(buffer + offset, &value, sizeof(LpfBandwidth));
        offset += sizeof(LpfBandwidth);
        break;
    }
    case SettingType::ENUM_IMU_ROTATION:
    {
        ImuRotation value = *(ImuRotation *)setting->value;
        memcpy(buffer + offset, &value, sizeof(ImuRotation));
        offset += sizeof(ImuRotation);
        break;
    }
    case SettingType::ENUM_DSHOT_MODE:
    {
        dshot_mode_t value = *(dshot_mode_t *)setting->value;
        memcpy(buffer + offset, &value, sizeof(dshot_mode_t));
        offset += sizeof(dshot_mode_t);
        break;
    }
    case SettingType::STRING:
    {
        String *value = static_cast<String *>(setting->value);
        strcpy((char *)(buffer + offset), value->c_str());
        offset += value->length() + 1; // +1 for null terminator
        break;
    }
    case SettingType::ENUM_RX_CHANNEL_MAP:
        // Not yet implemented for MSP (handled by dedicated commands)
        break;
    }
    return offset;
}

SetResult CommunicationManager::_deserializeMspPayloadToSetting(const uint8_t *payload, uint16_t payloadSize, Setting *setting)
{
    uint16_t offset = 0;

    // Read setting name (null-terminated string)
    uint16_t receivedNameLength = 0;
    while (offset + receivedNameLength < payloadSize && payload[offset + receivedNameLength] != '\0') {
        receivedNameLength++;
    }
    String receivedSettingName = _convertPayloadToString(payload + offset, receivedNameLength);
    offset += receivedNameLength + 1; // +1 for null terminator

    // Check if there's enough payload left for the setting type
    if (payloadSize - offset < sizeof(uint8_t)) {
        return SetResult::INVALID_FORMAT;
    }
    // Read setting type (1 byte)
    SettingType receivedSettingType = (SettingType)payload[offset++];

    // Check if the received setting name and type match the provided setting
    if (!receivedSettingName.equalsIgnoreCase(setting->name) || receivedSettingType != setting->type)
    {
        return SetResult::UNKNOWN_PARAMETER; // Or a more specific error like INVALID_FORMAT
    }

    // Deserialize setting value based on type
    switch (setting->type)
    {
    case SettingType::FLOAT:
    {
        if (payloadSize - offset < sizeof(float)) return SetResult::INVALID_FORMAT;
        float value;
        memcpy(&value, payload + offset, sizeof(float));
        *(float *)setting->value = value * setting->scaleFactor;
        break;
    }
    case SettingType::INT:
    {
        if (payloadSize - offset < sizeof(int)) return SetResult::INVALID_FORMAT;
        int value;
        memcpy(&value, payload + offset, sizeof(int));
        // Apply scaling to match firmware's internal PID_SCALE_FACTOR
        if (setting->scaleFactor == PID_SCALE_FACTOR) {
            *(int *)setting->value = value * PID_DISPLAY_SCALE_FACTOR;
        } else {
            *(int *)setting->value = value;
        }
        break;
    }
    case SettingType::UINT8:
    {
        if (payloadSize - offset < sizeof(uint8_t)) return SetResult::INVALID_FORMAT;
        uint8_t value;
        memcpy(&value, payload + offset, sizeof(uint8_t));
        *(uint8_t *)setting->value = value;
        break;
    }
    case SettingType::UINT16:
    {
        if (payloadSize - offset < sizeof(uint16_t)) return SetResult::INVALID_FORMAT;
        uint16_t value;
        memcpy(&value, payload + offset, sizeof(uint16_t));
        *(uint16_t *)setting->value = value;
        break;
    }
    case SettingType::ULONG:
    {
        if (payloadSize - offset < sizeof(unsigned long)) return SetResult::INVALID_FORMAT;
        unsigned long value;
        memcpy(&value, payload + offset, sizeof(unsigned long));
        *(unsigned long *)setting->value = value;
        break;
    }
    case SettingType::BOOL:
    {
        if (payloadSize - offset < sizeof(uint8_t)) return SetResult::INVALID_FORMAT; // bool is 1 byte
        bool value = (bool)payload[offset];
        *(bool *)setting->value = value;
        break;
    }
    case SettingType::ENUM_IBUS_PROTOCOL:
    {
        if (payloadSize - offset < sizeof(ReceiverProtocol)) return SetResult::INVALID_FORMAT;
        ReceiverProtocol value;
        memcpy(&value, payload + offset, sizeof(ReceiverProtocol));
        *(ReceiverProtocol *)setting->value = value;
        break;
    }
    case SettingType::ENUM_IMU_PROTOCOL:
    {
        if (payloadSize - offset < sizeof(ImuProtocol)) return SetResult::INVALID_FORMAT;
        ImuProtocol value;
        memcpy(&value, payload + offset, sizeof(ImuProtocol));
        *(ImuProtocol *)setting->value = value;
        break;
    }
    case SettingType::ENUM_LPF_BANDWIDTH:
    {
        if (payloadSize - offset < sizeof(LpfBandwidth)) return SetResult::INVALID_FORMAT;
        LpfBandwidth value;
        memcpy(&value, payload + offset, sizeof(LpfBandwidth));
        *(LpfBandwidth *)setting->value = value;
        break;
    }
    case SettingType::ENUM_IMU_ROTATION:
    {
        if (payloadSize - offset < sizeof(ImuRotation)) return SetResult::INVALID_FORMAT;
        ImuRotation value;
        memcpy(&value, payload + offset, sizeof(ImuRotation));
        *(ImuRotation *)setting->value = value;
        break;
    }
    case SettingType::ENUM_DSHOT_MODE:
    {
        if (payloadSize - offset < sizeof(dshot_mode_t)) return SetResult::INVALID_FORMAT;
        dshot_mode_t value;
        memcpy(&value, payload + offset, sizeof(dshot_mode_t));
        *(dshot_mode_t *)setting->value = value;
        break;
    }
    case SettingType::STRING:
    {
        // For strings, we read the remaining payload, so no fixed size check here
        String *value = static_cast<String *>(setting->value);
        *value = _convertPayloadToString(payload + offset, payloadSize - offset); // Read remaining payload as string
        break;
    }
    case SettingType::ENUM_RX_CHANNEL_MAP:
        // Not yet implemented for MSP (handled by dedicated commands)
        return SetResult::UNKNOWN_PARAMETER;
    }
    return SetResult::SUCCESS;
}

uint16_t CommunicationManager::_serializeReceiverChannelMapToMspPayload(uint8_t *buffer) const
{
    uint16_t offset = 0;
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
    {
        buffer[offset++] = settings.receiver.channelMapping.channel[i];
    }
    return offset;
}

SetResult CommunicationManager::_deserializeMspPayloadToReceiverChannelMap(const uint8_t *payload, uint16_t payloadSize)
{
    if (payloadSize != NUM_FLIGHT_CONTROL_INPUTS)
    {
        return SetResult::INVALID_FORMAT;
    }

    uint16_t offset = 0;
    for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
    {
        settings.receiver.channelMapping.channel[i] = payload[offset++];
    }
    return SetResult::SUCCESS;
}

uint16_t CommunicationManager::_serializeFlightStatusToMspPayload(uint8_t *buffer) const
{
    uint16_t offset = 0;

    // Loop Time (unsigned long)
    unsigned long loopTimeUs = _flightController->state.loopTimeUs;
    memcpy(buffer + offset, &loopTimeUs, sizeof(unsigned long));
    offset += sizeof(unsigned long);

    // CPU Load (float)
    float cpuLoad = _flightController->state.cpuLoad;
    memcpy(buffer + offset, &cpuLoad, sizeof(float));
    offset += sizeof(float);

    // Battery Voltage (float)
    float voltage = _flightController->state.voltage;
    memcpy(buffer + offset, &voltage, sizeof(float));
    offset += sizeof(float);

    // Current Draw (float)
    float current = _flightController->state.current;
    memcpy(buffer + offset, &current, sizeof(float));
    offset += sizeof(float);

    buffer[offset++] = (uint8_t)_flightController->state.isArmed;

    buffer[offset++] = (uint8_t)_flightController->state.isFailsafeActive;

    buffer[offset++] = (uint8_t)_flightController->state.currentFlightMode;

    return offset;
}

uint16_t CommunicationManager::_serializeFirmwareVersionToMspPayload(uint8_t *buffer) const
{
    uint16_t offset = 0;
    char tempBuffer[VERSION_STRING_BUFFER_SIZE]; // Sufficient for version and build ID

    // Firmware Version (string)
    snprintf(tempBuffer, sizeof(tempBuffer), "%d", FIRMWARE_VERSION);
    strcpy((char *)(buffer + offset), tempBuffer);
    offset += strlen(tempBuffer) + 1;

    // Build ID (string)
    snprintf(tempBuffer, sizeof(tempBuffer), "%d", FIRMWARE_BUILD_ID);
    strcpy((char *)(buffer + offset), tempBuffer);
    offset += strlen(tempBuffer) + 1;

    return offset;
}

uint16_t CommunicationManager::_serializeLiveFlightDataToMspPayload(uint8_t *buffer) const
{
    uint16_t offset = 0;

    // Attitude (roll, pitch, yaw - float)
    memcpy(buffer + offset, &_flightController->state.attitude.roll, sizeof(float));
    offset += sizeof(float);
    memcpy(buffer + offset, &_flightController->state.attitude.pitch, sizeof(float));
    offset += sizeof(float);
    memcpy(buffer + offset, &_flightController->state.attitude.yaw, sizeof(float));
    offset += sizeof(float);

    // Status (isArmed, isFailsafeActive, currentFlightMode - bool, bool, uint8_t)
    buffer[offset++] = (uint8_t)_flightController->state.isArmed;
    buffer[offset++] = (uint8_t)_flightController->state.isFailsafeActive;
    buffer[offset++] = (uint8_t)_flightController->state.currentFlightMode;

    // Loop Time (unsigned long)
    memcpy(buffer + offset, &_flightController->state.loopTimeUs, sizeof(unsigned long));
    offset += sizeof(unsigned long);

    // Motor Outputs (NUM_MOTORS * uint16_t)
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        memcpy(buffer + offset, &_flightController->state.motorOutputs[i], sizeof(uint16_t));
        offset += sizeof(uint16_t);
    }

    // Receiver Channels (RECEIVER_CHANNEL_COUNT * uint16_t)
    for (int i = 0; i < RECEIVER_CHANNEL_COUNT; ++i)
    {
        memcpy(buffer + offset, &_flightController->state.receiverChannels[i], sizeof(uint16_t));
        offset += sizeof(uint16_t);
    }

    return offset;
}

// --- Refactoring: Settings Registry ---

const Setting CommunicationManager::settingsRegistry[] = {
    {"pid.roll.kp", SettingType::INT, &settings.pid.roll.kp, PID_SCALE_FACTOR},
    {"pid.roll.ki", SettingType::INT, &settings.pid.roll.ki, PID_SCALE_FACTOR},
    {"pid.roll.kd", SettingType::INT, &settings.pid.roll.kd, PID_SCALE_FACTOR},
    {"pid.pitch.kp", SettingType::INT, &settings.pid.pitch.kp, PID_SCALE_FACTOR},
    {"pid.pitch.ki", SettingType::INT, &settings.pid.pitch.ki, PID_SCALE_FACTOR},
    {"pid.pitch.kd", SettingType::INT, &settings.pid.pitch.kd, PID_SCALE_FACTOR},
    {"pid.yaw.kp", SettingType::INT, &settings.pid.yaw.kp, PID_SCALE_FACTOR},
    {"pid.yaw.ki", SettingType::INT, &settings.pid.yaw.ki, PID_SCALE_FACTOR},
    {"pid.yaw.kd", SettingType::INT, &settings.pid.yaw.kd, PID_SCALE_FACTOR},
    {"pid.integral_limit", SettingType::FLOAT, &settings.pid.integralLimit, DEFAULT_SCALE_FACTOR},
    {"rates.angle", SettingType::FLOAT, &settings.flightRates.maxAngleRollPitch, DEFAULT_SCALE_FACTOR},
    {"rates.yaw", SettingType::FLOAT, &settings.flightRates.maxRateYaw, DEFAULT_SCALE_FACTOR},
    {"rates.acro", SettingType::FLOAT, &settings.flightRates.maxRateRollPitch, DEFAULT_SCALE_FACTOR},
    {"filter.comp_tau", SettingType::FLOAT, &settings.filter.complementaryFilterTau, DEFAULT_SCALE_FACTOR},
    {"gyro.lpf_cutoff_freq", SettingType::FLOAT, &settings.filter.gyroLpfCutoffFreq, DEFAULT_SCALE_FACTOR},
    {"accel.lpf_cutoff_freq", SettingType::FLOAT, &settings.filter.accelLpfCutoffFreq, DEFAULT_SCALE_FACTOR},
    {"gyro.lpf_stages", SettingType::UINT8, &settings.filter.gyroLpfStages, DEFAULT_SCALE_FACTOR},
    {"accel.lpf_stages", SettingType::UINT8, &settings.filter.accelLpfStages, DEFAULT_SCALE_FACTOR},
    {"filter.sample_freq", SettingType::FLOAT, &settings.filter.filterSampleFreq, DEFAULT_SCALE_FACTOR},
    {"gyro.notch.enable", SettingType::BOOL, &settings.filter.enableGyroNotchFilter, DEFAULT_SCALE_FACTOR},
    {"gyro.notch.freq", SettingType::FLOAT, &settings.filter.gyroNotchFreq, DEFAULT_SCALE_FACTOR},
    {"gyro.notch.q", SettingType::FLOAT, &settings.filter.gyroNotchQ, DEFAULT_SCALE_FACTOR},
    {"rx.min", SettingType::UINT16, &settings.receiver.minValue, DEFAULT_SCALE_FACTOR},
    {"rx.max", SettingType::UINT16, &settings.receiver.maxValue, DEFAULT_SCALE_FACTOR},
    {"rx.arming_threshold", SettingType::UINT16, &settings.receiver.armingThreshold, DEFAULT_SCALE_FACTOR},
    {"rx.failsafe_threshold", SettingType::UINT16, &settings.receiver.failsafeThreshold, DEFAULT_SCALE_FACTOR},
    {"rx.protocol", SettingType::ENUM_IBUS_PROTOCOL, &settings.receiver.protocol, DEFAULT_SCALE_FACTOR},
    {"imu.protocol", SettingType::ENUM_IMU_PROTOCOL, &settings.imu.protocol, DEFAULT_SCALE_FACTOR},
    {"imu.lpf", SettingType::ENUM_LPF_BANDWIDTH, &settings.imu.lpfBandwidth, DEFAULT_SCALE_FACTOR},
    {"imu.rotation", SettingType::ENUM_IMU_ROTATION, &settings.imu.rotation, DEFAULT_SCALE_FACTOR},
    {"motor.idle_speed", SettingType::FLOAT, &settings.motor.idleSpeedPercent, DEFAULT_SCALE_FACTOR},
    {"motor.dshot_mode", SettingType::ENUM_DSHOT_MODE, &settings.motor.dshotMode, DEFAULT_SCALE_FACTOR},
    {"enforce_loop_time", SettingType::BOOL, &settings.logging.enforceLoopTime, DEFAULT_SCALE_FACTOR},
    {"cal.mpu_readings", SettingType::INT, &settings.calibration.mpuCalibrationReadings, DEFAULT_SCALE_FACTOR},
    {"cal.accel_z_g", SettingType::FLOAT, &settings.calibration.accelZGravity, DEFAULT_SCALE_FACTOR},
    {"log.print_interval", SettingType::ULONG, &settings.logging.printIntervalMs, DEFAULT_SCALE_FACTOR},
    {"log.enable", SettingType::BOOL, &settings.logging.enableLogging, DEFAULT_SCALE_FACTOR},
    {"log.debug.en", SettingType::BOOL, &settings.logging.enableDebugLogging, DEFAULT_SCALE_FACTOR}, // Add this line
    {"log.test_string", SettingType::STRING, &settings.logging.testString, DEFAULT_SCALE_FACTOR},
    {"bench.run.en", SettingType::BOOL, &settings.logging.enableBenchRunMode, DEFAULT_SCALE_FACTOR},
};const int CommunicationManager::numSettings = sizeof(CommunicationManager::settingsRegistry) / sizeof(Setting);

CommunicationManager *CommunicationManager::_communicationManagerInstance = nullptr;

// --- Helper Functions for Parsing and Validation ---

SetResult CommunicationManager::_parseAndValidateFloatSetting(const String &valueStr, float &outValue, float scaleFactor, String &expectedValue) const
{
    float val = valueStr.toFloat();
    if (valueStr.length() > 0 && val == 0.0f && valueStr != "0" && valueStr != "0.0")
    {
        expectedValue = "float";
        return SetResult::INVALID_VALUE;
    }
    outValue = val * scaleFactor;
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateIntSetting(const String &valueStr, int &outValue, String &expectedValue) const
{
    long val = valueStr.toInt();
    if (valueStr.length() > 0 && val == 0 && valueStr != "0")
    {
        expectedValue = "integer";
        return SetResult::INVALID_VALUE;
    }
    // No explicit range check for int, as it can be negative.
    // If specific int ranges are needed, they should be added here.
    outValue = (int)val;
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateUint16Setting(const String &valueStr, uint16_t &outValue, String &expectedValue) const
{
    long val = valueStr.toInt();
    if (valueStr.length() > 0 && val == 0 && valueStr != "0")
    {
        expectedValue = "integer";
        return SetResult::INVALID_VALUE;
    }
    else if (val < 0 || val > MAX_UINT16_VALUE)
    { // uint16_t range
        expectedValue = "0-" + String(MAX_UINT16_VALUE);
        return SetResult::OUT_OF_RANGE;
    }
    outValue = (uint16_t)val;
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateULongSetting(const String &valueStr, unsigned long &outValue, String &expectedValue) const
{
    // toULong() returns 0 if no valid conversion could be performed.
    // We need to check if the string was actually "0" or if it was invalid.
    unsigned long val = strtoul(valueStr.c_str(), NULL, 10);
    if (valueStr.length() > 0 && val == 0 && valueStr != "0")
    {
        expectedValue = "unsigned long integer";
        return SetResult::INVALID_VALUE;
    }
    outValue = val;
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateReceiverProtocolSetting(const String &valueStr, ReceiverProtocol &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("IBUS"))
        outValue = PROTOCOL_IBUS;
    else if (valueStr.equalsIgnoreCase("PPM"))
        outValue = PROTOCOL_PPM;
    else
    {
        expectedValue = "IBUS, PPM";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateImuProtocolSetting(const String &valueStr, ImuProtocol &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("MPU6050"))
        outValue = IMU_MPU6050;
    else
    {
        expectedValue = "MPU6050";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateLpfBandwidthSetting(const String &valueStr, LpfBandwidth &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("LPF_256HZ_N_0MS"))
        outValue = LPF_256HZ_N_0MS;
    else if (valueStr.equalsIgnoreCase("LPF_188HZ_N_2MS"))
        outValue = LPF_188HZ_N_2MS;
    else if (valueStr.equalsIgnoreCase("LPF_98HZ_N_3MS"))
        outValue = LPF_98HZ_N_3MS;
    else if (valueStr.equalsIgnoreCase("LPF_42HZ_N_5MS"))
        outValue = LPF_42HZ_N_5MS;
    else if (valueStr.equalsIgnoreCase("LPF_20HZ_N_10MS"))
        outValue = LPF_20HZ_N_10MS;
    else if (valueStr.equalsIgnoreCase("LPF_10HZ_N_13MS"))
        outValue = LPF_10HZ_N_13MS;
    else if (valueStr.equalsIgnoreCase("LPF_5HZ_N_18MS"))
        outValue = LPF_5HZ_N_18MS;
    else
    {
        expectedValue = "LPF_256HZ_N_0MS, LPF_188HZ_N_2MS, LPF_98HZ_N_3MS, LPF_42HZ_N_5MS, LPF_20HZ_N_10MS, LPF_10HZ_N_13MS, LPF_5HZ_N_18MS";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateImuRotationSetting(const String &valueStr, ImuRotation &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("NONE"))
        outValue = IMU_ROTATION_NONE;
    else if (valueStr.equalsIgnoreCase("90_CW"))
        outValue = IMU_ROTATION_90_DEG_CW;
    else if (valueStr.equalsIgnoreCase("180_CW"))
        outValue = IMU_ROTATION_180_DEG_CW;
    else if (valueStr.equalsIgnoreCase("270_CW"))
        outValue = IMU_ROTATION_270_DEG_CW;
    else if (valueStr.equalsIgnoreCase("FLIP"))
        outValue = IMU_ROTATION_FLIP;
    else
    {
        expectedValue = "NONE, 90_CW, 180_CW, 270_CW, FLIP";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateDShotModeSetting(const String &valueStr, dshot_mode_t &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("DSHOT_OFF"))
        outValue = DSHOT_OFF;
    else if (valueStr.equalsIgnoreCase("DSHOT150"))
        outValue = DSHOT150;
    else if (valueStr.equalsIgnoreCase("DSHOT300"))
        outValue = DSHOT300;
    else if (valueStr.equalsIgnoreCase("DSHOT600"))
        outValue = DSHOT600;
    else if (valueStr.equalsIgnoreCase("DSHOT1200"))
        outValue = DSHOT1200;
    else
    {
        expectedValue = "DSHOT_OFF, DSHOT150, DSHOT300, DSHOT600, DSHOT1200";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateBoolSetting(const String &valueStr, bool &outValue, String &expectedValue) const
{
    if (valueStr.equalsIgnoreCase("true"))
        outValue = true;
    else if (valueStr.equalsIgnoreCase("false"))
        outValue = false;
    else
    {
        expectedValue = "true, false";
        return SetResult::INVALID_VALUE;
    }
    return SetResult::SUCCESS;
}

SetResult CommunicationManager::_parseAndValidateReceiverChannelMapSetting(const String &param, const String &valueStr, int &outValue, String &expectedValue) const
{
    String inputName = param.substring(RX_MAP_PREFIX_LENGTH);
    int channelValue = valueStr.toInt();

    if (valueStr.length() > 0 && channelValue == 0 && valueStr != "0")
    {
        expectedValue = "integer";
        return SetResult::INVALID_VALUE;
    }
    else if (channelValue < 0 || channelValue >= RECEIVER_CHANNEL_COUNT)
    {
        expectedValue = "0-" + String(RECEIVER_CHANNEL_COUNT - 1);
        return SetResult::OUT_OF_RANGE;
    }
    else
    {
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
        {
            if (inputName.equalsIgnoreCase(_getFlightControlInputName((FlightControlInput)i)))
            {
                outValue = channelValue;
                return SetResult::SUCCESS;
            }
        }
    }
    return SetResult::UNKNOWN_PARAMETER;
}





void CommunicationManager::_sendMspMessageResponse(uint16_t command, const uint8_t *payload, uint16_t payloadSize)
{
    _mspMessageParser.sendMspMessage(Serial, '>', command, payload, payloadSize, true);
}



// --- Public Methods ---

CommunicationManager::CommunicationManager(FlightController *flightController) : _flightController(flightController)
{
    _communicationManagerInstance = this;
    _mspMessageParser.begin(Serial, "USB");
    _mspMessageParser.onMessage(_onMspMessageReceivedCallback);
}

void CommunicationManager::_onMspMessageReceivedCallback(const MspMessage &message, const char *prefix)
{
    if (_communicationManagerInstance)
    {
        String debugMsg = "Received MSP command: 0x" + String(message.command, HEX) +
                          ", Size: " + String(message.payloadSize) +
                          ", Direction: '" + String(message.direction) + "'" +
                          " from prefix: " + String(prefix);
        _communicationManagerInstance->_processMspCommand(message);
    }
}

void CommunicationManager::_processMspCommand(const MspMessage &message)
{
    switch (message.command)
    {
    case MSP_FC_GET_SETTING:
    {
        String settingName = _convertPayloadToString(message.payload, message.payloadSize);

        if (settingName.equalsIgnoreCase("all"))
        {
            // Handle request for all settings by sending them one by one
            for (int i = 0; i < numSettings; ++i)
            {
                const Setting *setting = &settingsRegistry[i];
                uint8_t responsePayload[MSP_MAX_PAYLOAD_SIZE_SETTINGS]; // Increased buffer for safety
                uint16_t payloadSize = _serializeSettingToMspPayload(setting, responsePayload);
                _sendMspMessageResponse(MSP_FC_SETTING_RESPONSE, responsePayload, payloadSize);
            }
            _isTelemetryStreamingEnabled = true; // Enable live data streaming after sending all settings
        }
        else
        {
            // Handle request for a single setting
            const Setting *foundSetting = nullptr;
            for (int i = 0; i < numSettings; ++i)
            {
                if (settingName.equalsIgnoreCase(settingsRegistry[i].name))
                {
                    foundSetting = &settingsRegistry[i];
                    break;
                }
            }

            if (foundSetting)
            {
                uint8_t responsePayload[MSP_MAX_PAYLOAD_SIZE_SETTINGS];
                uint16_t payloadSize = _serializeSettingToMspPayload(foundSetting, responsePayload);
                _sendMspMessageResponse(MSP_FC_SETTING_RESPONSE, responsePayload, payloadSize);
            }
            else
            {
                String errorMessage = "Unknown setting: " + settingName;
                uint8_t errorPayload[errorMessage.length() + 1];
                strcpy((char *)errorPayload, errorMessage.c_str());
                _sendMspMessageResponse(MSP_FC_ERROR, errorPayload, errorMessage.length() + 1);
            }
        }
        break;
    }
    case MSP_FC_SET_SETTING:
    {
        // Correctly extract the setting name from the payload
        uint16_t nameLength = 0;
        while (nameLength < message.payloadSize && message.payload[nameLength] != '\0') {
            nameLength++;
        }
        String settingName = _convertPayloadToString(message.payload, nameLength);

        const Setting *foundSetting = nullptr;
        for (int i = 0; i < numSettings; ++i)
        {
            if (settingName.equalsIgnoreCase(settingsRegistry[i].name))
            {
                foundSetting = &settingsRegistry[i];
                break;
            }
        }

        if (foundSetting)
        {
            SetResult result = _deserializeMspPayloadToSetting(message.payload, message.payloadSize, (Setting *)foundSetting);
            if (result == SetResult::SUCCESS)
            {
                uint8_t responsePayload[1];
                responsePayload[0] = (uint8_t)SetResult::SUCCESS;
                _sendMspMessageResponse(MSP_FC_SETTING_RESPONSE, responsePayload, MSP_PAYLOAD_SIZE_STATUS);
            }
            else
            {
                String errorMessage = "Failed to update setting: " + String((int)result);
                String fullErrorMessage = "Failed to set " + settingName + ": Error " + String((int)result);
                uint8_t errorPayload[fullErrorMessage.length() + 1];
                strcpy((char*)errorPayload, fullErrorMessage.c_str());
                _sendMspMessageResponse(MSP_FC_ERROR, errorPayload, fullErrorMessage.length() + 1);
            }
        }
        else
        {
            String errorMessage = "Setting not found: " + settingName;
            String fullErrorMessage = "Unknown setting: " + settingName;
            uint8_t errorPayload[fullErrorMessage.length() + 1];
            strcpy((char*)errorPayload, fullErrorMessage.c_str());
            _sendMspMessageResponse(MSP_FC_ERROR, errorPayload, fullErrorMessage.length() + 1);
        }
        break;
    }
    case MSP_FC_GET_RX_MAP:
    {
        uint8_t responsePayload[NUM_FLIGHT_CONTROL_INPUTS];
        uint16_t payloadSize = _serializeReceiverChannelMapToMspPayload(responsePayload);
        _sendMspMessageResponse(MSP_FC_SETTING_RESPONSE, responsePayload, payloadSize);
        break;
    }
    case MSP_FC_SET_RX_MAP:
    {
        SetResult result = _deserializeMspPayloadToReceiverChannelMap(message.payload, message.payloadSize);
        if (result == SetResult::SUCCESS)
        {
                    uint8_t responsePayload[MSP_PAYLOAD_SIZE_STATUS];
                    responsePayload[0] = (uint8_t)SetResult::SUCCESS;
                    _sendMspMessageResponse(MSP_FC_SETTING_RESPONSE, responsePayload, MSP_PAYLOAD_SIZE_STATUS);        }
        else
        {
            String errorMessage = "Failed to update RX Channel Map: " + String((int)result);
            String fullErrorMessage = "Failed to set RX Map: Error " + String((int)result);
            uint8_t errorPayload[fullErrorMessage.length() + 1];
            strcpy((char*)errorPayload, fullErrorMessage.c_str());
            _sendMspMessageResponse(MSP_FC_ERROR, errorPayload, fullErrorMessage.length() + 1);
        }
        break;
    }
    case MSP_FC_SAVE_SETTINGS:
    {
        saveSettings([this](const String& msg){ /* No debug message */ });
        uint8_t responsePayload[MSP_PAYLOAD_SIZE_STATUS] = {(uint8_t)SetResult::SUCCESS};
        _sendMspMessageResponse(MSP_FC_SETTING_RESPONSE, responsePayload, MSP_PAYLOAD_SIZE_STATUS);
        break;
    }
    case MSP_FC_RESET_SETTINGS:
    {
        settings = FlightControllerSettings();
        saveSettings([this](const String& msg){ /* No debug message */ });
        uint8_t responsePayload[MSP_PAYLOAD_SIZE_STATUS] = {(uint8_t)SetResult::SUCCESS};
        _sendMspMessageResponse(MSP_FC_SETTING_RESPONSE, responsePayload, MSP_PAYLOAD_SIZE_STATUS);
        break;
    }
    case MSP_FC_REBOOT:
    {
        ESP.restart();
        break;
    }
    case MSP_FC_CALIBRATE_IMU:
    {
        _flightController->requestImuCalibration();
        uint8_t responsePayload[MSP_PAYLOAD_SIZE_STATUS] = {(uint8_t)SetResult::SUCCESS};
        _sendMspMessageResponse(MSP_FC_SETTING_RESPONSE, responsePayload, MSP_PAYLOAD_SIZE_STATUS);
        break;
    }
    case MSP_FC_GET_STATUS:
    {
        uint8_t responsePayload[MSP_MAX_PAYLOAD_SIZE_STATUS_VERSION]; // Max payload size for status
        uint16_t payloadSize = _serializeFlightStatusToMspPayload(responsePayload);
        _sendMspMessageResponse(MSP_FC_SETTING_RESPONSE, responsePayload, payloadSize);
        break;
    }
    case MSP_FC_GET_VERSION:
    {
        uint8_t responsePayload[MSP_MAX_PAYLOAD_SIZE_STATUS_VERSION]; // Max payload size for version
        uint16_t payloadSize = _serializeFirmwareVersionToMspPayload(responsePayload);
        _sendMspMessageResponse(MSP_FC_SETTING_RESPONSE, responsePayload, payloadSize);
        break;
    }
    default:
    {
        String errorMessage = "Unhandled MSP Command: " + String(message.command) + " with payload size " + String(message.payloadSize);
        uint8_t errorPayload[errorMessage.length() + 1];
        strcpy((char*)errorPayload, errorMessage.c_str());
        _sendMspMessageResponse(MSP_FC_ERROR, errorPayload, errorMessage.length() + 1);
        break;
    }
    }
}

void CommunicationManager::initializeCommunication() {}

void CommunicationManager::processCommunication()
{
    if (_currentOperatingMode == OperatingMode::MSP_API) {
        _processMspApiInput(); // MspParser handles input in MSP_API mode
    } else {
        _processSerialInput(); // Handle string input for FLIGHT and CLI modes
    }

    if (_currentOperatingMode == OperatingMode::MSP_API && settings.logging.enableLogging && _isTelemetryStreamingEnabled && millis() - _lastTelemetrySendTimeUs >= settings.logging.printIntervalMs)
    {
        uint8_t responsePayload[MSP_MAX_PAYLOAD_SIZE_FLIGHT_STATUS]; // Max payload size for flight status
        uint16_t payloadSize = _serializeLiveFlightDataToMspPayload(responsePayload);
        _sendMspMessageResponse(MSP_FC_LIVE_DATA, responsePayload, payloadSize);
        _lastTelemetrySendTimeUs = millis();
    }
}

void CommunicationManager::_processFlightModeInput(const String &input)
{
    if (input.equalsIgnoreCase("cli"))
    {
        _currentOperatingMode = OperatingMode::CLI;
        settings.logging.enableLogging = false;
        settings.logging.inCliMode = true; // Set in_cli_mode to true
        Serial.println("--- CLI  Activated ---");
        Serial.print("ESP32_FC > ");
    }
    else if (input.startsWith("$"))
    {
        _currentOperatingMode = OperatingMode::MSP_API;
        settings.logging.enableLogging = true;
        settings.logging.inCliMode = false;
        settings.logging.enableDebugLogging = true; // Enable debug logging for MSP API
        Serial.println("--- MSP API Activated ---");
    }
}

// --- Private Methods: Main Logic ---

void CommunicationManager::_processSerialInput()
{
    if (_currentOperatingMode == OperatingMode::MSP_API) {
        return; // MspParser handles input in MSP_API mode
    }

    if (Serial.available() == 0)
        return;

    // Peek at the first byte to check for MSP message
    char peekedChar = Serial.peek();
    if (peekedChar == '$') {
        // If it's an MSP message, switch to MSP_API mode and let MspParser handle it
        _currentOperatingMode = OperatingMode::MSP_API;
        settings.logging.enableLogging = true; // Enable logging for MSP API
        return;
    }

    // If not an MSP message, proceed with string input

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0)
        return;

    switch (_currentOperatingMode)
    {
    case OperatingMode::FLIGHT:
        _processFlightModeInput(input);
        break;
    case OperatingMode::CLI:
    {
        String commandName = (input.indexOf(' ') != -1) ? input.substring(0, input.indexOf(' ')) : input;
        if (commandName.equalsIgnoreCase("exit"))
        {
            _currentOperatingMode = OperatingMode::FLIGHT;
            settings.logging.inCliMode = false; // Set in_cli_mode to false
            Serial.println("--- CLI  Deactivated ---");
            return;
        }
        _executeCliCommand(input, false);
        if (!commandName.equalsIgnoreCase("save") && !commandName.equalsIgnoreCase("reboot") && !commandName.equalsIgnoreCase("reset"))
        {
            if (settings.logging.inCliMode) Serial.print("ESP32_FC > ");
        }
        break;
    }
    }
}

void CommunicationManager::_processMspApiInput()
{
    _mspMessageParser.update();
}

void CommunicationManager::_executeCliCommand(String command, bool isApiMode)
{
    String commandName = "";
    String commandArgs = "";
    int firstSpace = command.indexOf(' ');
    if (firstSpace != -1)
    {
        commandName = command.substring(0, firstSpace);
        commandArgs = command.substring(firstSpace + 1);
    }
    else
    {
        commandName = command;
    }
    commandName.toLowerCase(); // Only lowercase the command name itself

    if (commandName.equals("dump"))
    {
        _displayAllSettingsCliCommand();
    }
    else if (commandName.equals("save") || commandName.equals("reset") || commandName.equals("reboot") || commandName.equals("status") || commandName.equals("version"))
    {
        _processSystemCliCommand(commandName, isApiMode);
    }
    else if (commandName.equals("calibrate_imu") || commandName.equals("help"))
    {
        _processUtilityCliCommand(commandName, isApiMode);
    }
    else if (commandName.equals("get") || commandName.equals("set"))
    {
        _processGetSetCliCommand(commandName, commandArgs, isApiMode);
    }
    else
    {
        if (settings.logging.inCliMode) {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        }
    }
}

// --- New Helper Methods for Command Handling ---

void CommunicationManager::_processSystemCliCommand(String commandName, bool isApiMode)
{
    if (commandName.equals("save"))
    {
        if (settings.logging.inCliMode)
            Serial.println("INFO: Settings saved. Rebooting...");
        saveSettings([](const String&){});
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    }
    else if (commandName.equals("reset"))
    {
        if (settings.logging.inCliMode)
            Serial.println("INFO: All settings have been reset to their default values and saved.");
        settings = FlightControllerSettings();
        saveSettings([](const String&){});
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    }
    else if (commandName.equals("reboot"))
    {
        if (settings.logging.inCliMode)
            Serial.println("Rebooting...");
        delay(CLI_REBOOT_DELAY_MS);
        ESP.restart();
    }
    else if (commandName.equals("status"))
    {
        _displaySystemStatusCliCommand();
    }
    else if (commandName.equals("version"))
    {
        _displayFirmwareVersionCliCommand();
    }
    else
    {
        if (settings.logging.inCliMode) {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        }
    }
}

void CommunicationManager::_processUtilityCliCommand(String commandName, bool isApiMode)
{
    if (commandName.equals("calibrate_imu"))
    {
        if (settings.logging.inCliMode)
            Serial.println("INFO: IMU calibration requested.");
        _flightController->requestImuCalibration();
    }
    else if (commandName.equals("help"))
    {
        _displayCliHelp();
    }
    else
    {
        if (settings.logging.inCliMode) {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        }
    }
}

void CommunicationManager::_processGetSetCliCommand(String commandName, String commandArgs, bool isApiMode)
{
    if (commandName.equals("get"))
    {
        _processGetSettingCliCommand(commandArgs, isApiMode);
    }
    else if (commandName.equals("set"))
    {
        _processSetSettingCliCommand(commandArgs, isApiMode);
    }
    else
    {
        if (settings.logging.inCliMode) {
            Serial.print("Unknown command: ");
            Serial.println(commandName);
        }
    }
}

void CommunicationManager::_processGetSettingCliCommand(String settingName, bool isApiMode)
{
    const Setting *foundSetting = nullptr;
    for (int i = 0; i < numSettings; ++i)
    {
        if (settingName.equalsIgnoreCase(settingsRegistry[i].name))
        {
            foundSetting = &settingsRegistry[i];
            break;
        }
    }

    if (foundSetting)
    {
        if (settings.logging.inCliMode) {
            Serial.print(foundSetting->name);
            Serial.print(": ");
            switch (foundSetting->type)
            {
            case SettingType::FLOAT:
                Serial.println(*(float *)foundSetting->value / foundSetting->scaleFactor, 4);
                break;
            case SettingType::INT:
                if (foundSetting->scaleFactor == PID_SCALE_FACTOR)
                {
                    Serial.println(*(int *)foundSetting->value / PID_DISPLAY_SCALE_FACTOR);
                }
                else
                {
                    Serial.println(*(int *)foundSetting->value);
                }
                break;
            case SettingType::UINT8:
                Serial.println(*(uint8_t *)foundSetting->value);
                break;
            case SettingType::UINT16:
                Serial.println(*(uint16_t *)foundSetting->value);
                break;
            case SettingType::ULONG:
                Serial.println(*(unsigned long *)foundSetting->value);
                break;
            case SettingType::ENUM_IBUS_PROTOCOL:
                Serial.println(_getReceiverProtocolName(*(ReceiverProtocol *)foundSetting->value));
                break;
            case SettingType::ENUM_IMU_PROTOCOL:
                Serial.println(_getImuProtocolName(*(ImuProtocol *)foundSetting->value));
                break;
            case SettingType::ENUM_LPF_BANDWIDTH:
                Serial.println(_getLpfBandwidthName(*(LpfBandwidth *)foundSetting->value));
                break;
            case SettingType::ENUM_IMU_ROTATION:
                Serial.println(_getImuRotationName(*(ImuRotation *)foundSetting->value));
                break;
            case SettingType::BOOL:
                Serial.println(_getBoolName(*(bool *)foundSetting->value));
                break;
            case SettingType::ENUM_DSHOT_MODE:
                Serial.println(_getDShotModeName(*(dshot_mode_t *)foundSetting->value));
                break;
            case SettingType::STRING:
                Serial.println(*(String *)foundSetting->value);
                break;
            case SettingType::ENUM_RX_CHANNEL_MAP:
                Serial.println("Use 'dump' to see RX channel map.");
                break;
            }
        }
    }
    else
    {
        if (settings.logging.inCliMode) {
            Serial.print("Unknown setting: ");
            Serial.println(settingName);
        }
    }
}

void CommunicationManager::_processSetSettingCliCommand(String commandArgs, bool isApiMode)
{
    int spaceIndex = commandArgs.indexOf(' ');
    if (spaceIndex == -1)
    {
        if (settings.logging.inCliMode) {
            Serial.println("ERROR: Invalid set command format. Usage: set <param> <value>");
        }
        return;
    }

    String settingName = commandArgs.substring(0, spaceIndex);
    String valueStr = commandArgs.substring(spaceIndex + 1);

    const Setting *foundSetting = nullptr;
    for (int i = 0; i < numSettings; ++i)
    {
        if (settingName.equalsIgnoreCase(settingsRegistry[i].name))
        {
            foundSetting = &settingsRegistry[i];
            break;
        }
    }

    if (foundSetting)
    {
        String expectedValueType = "";
        SetResult result = SetResult::SUCCESS;

        // Handle RX Channel Map separately
        if (settingName.startsWith(RX_MAP_PREFIX) && foundSetting->type == SettingType::ENUM_RX_CHANNEL_MAP)
        {
            int channelValue;
            result = _parseAndValidateReceiverChannelMapSetting(settingName, valueStr, channelValue, expectedValueType);
            if (result == SetResult::SUCCESS)
            {
                String inputName = settingName.substring(RX_MAP_PREFIX_LENGTH);
                for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
                {
                    if (inputName.equalsIgnoreCase(_getFlightControlInputName((FlightControlInput)i)))
                    {
                        settings.receiver.channelMapping.channel[i] = channelValue;
                        break;
                    }
                }
            }
        }
        else
        {
            // Handle other settings
            switch (foundSetting->type)
            {
            case SettingType::FLOAT:
            {
                float value;
                result = _parseAndValidateFloatSetting(valueStr, value, foundSetting->scaleFactor, expectedValueType);
                if (result == SetResult::SUCCESS)
                    *(float *)foundSetting->value = value;
                break;
            }
            case SettingType::INT:
            {
                int value;
                result = _parseAndValidateIntSetting(valueStr, value, expectedValueType);
                if (result == SetResult::SUCCESS)
                {
                    if (foundSetting->scaleFactor == PID_SCALE_FACTOR) {
                        *(int *)foundSetting->value = value * PID_DISPLAY_SCALE_FACTOR;
                    } else {
                        *(int *)foundSetting->value = value;
                    }
                }
                break;
            }
            case SettingType::UINT8:
            {
                long val = valueStr.toInt();
                if (valueStr.length() > 0 && val == 0 && valueStr != "0")
                {
                    expectedValueType = "integer";
                    result = SetResult::INVALID_VALUE;
                }
                else if (val < 0 || val > MAX_UINT8_VALUE)
                {
                    expectedValueType = "0-" + String(MAX_UINT8_VALUE);
                    result = SetResult::OUT_OF_RANGE;
                }
                else
                {
                    *(uint8_t *)foundSetting->value = (uint8_t)val;
                }
                break;
            }
            case SettingType::UINT16:
            {
                uint16_t value;
                result = _parseAndValidateUint16Setting(valueStr, value, expectedValueType);
                if (result == SetResult::SUCCESS)
                    *(uint16_t *)foundSetting->value = value;
                break;
            }
            case SettingType::ULONG:
            {
                unsigned long value;
                result = _parseAndValidateULongSetting(valueStr, value, expectedValueType);
                if (result == SetResult::SUCCESS)
                    *(unsigned long *)foundSetting->value = value;
                break;
            }
            case SettingType::BOOL:
            {
                bool value;
                result = _parseAndValidateBoolSetting(valueStr, value, expectedValueType);
                if (result == SetResult::SUCCESS)
                    *(bool *)foundSetting->value = value;
                break;
            }
            case SettingType::ENUM_IBUS_PROTOCOL:
            {
                ReceiverProtocol value;
                result = _parseAndValidateReceiverProtocolSetting(valueStr, value, expectedValueType);
                if (result == SetResult::SUCCESS)
                    *(ReceiverProtocol *)foundSetting->value = value;
                break;
            }
            case SettingType::ENUM_IMU_PROTOCOL:
            {
                ImuProtocol value;
                result = _parseAndValidateImuProtocolSetting(valueStr, value, expectedValueType);
                if (result == SetResult::SUCCESS)
                    *(ImuProtocol *)foundSetting->value = value;
                break;
            }
            case SettingType::ENUM_LPF_BANDWIDTH:
            {
                LpfBandwidth value;
                result = _parseAndValidateLpfBandwidthSetting(valueStr, value, expectedValueType);
                if (result == SetResult::SUCCESS)
                    *(LpfBandwidth *)foundSetting->value = value;
                break;
            }
            case SettingType::ENUM_IMU_ROTATION:
            {
                ImuRotation value;
                result = _parseAndValidateImuRotationSetting(valueStr, value, expectedValueType);
                if (result == SetResult::SUCCESS)
                    *(ImuRotation *)foundSetting->value = value;
                break;
            }
            case SettingType::ENUM_DSHOT_MODE:
            {
                dshot_mode_t value;
                result = _parseAndValidateDShotModeSetting(valueStr, value, expectedValueType);
                if (result == SetResult::SUCCESS)
                    *(dshot_mode_t *)foundSetting->value = value;
                break;
            }
            case SettingType::STRING:
            {
                String *value = static_cast<String *>(foundSetting->value);
                *value = valueStr;
                result = SetResult::SUCCESS;
                break;
            }
            case SettingType::ENUM_RX_CHANNEL_MAP:
                // Handled above
                break;
            }
        }

        if (settings.logging.inCliMode) {
            if (result == SetResult::SUCCESS)
            {
                Serial.print("INFO: Setting '");
                Serial.print(settingName);
                Serial.println("' updated.");
            }
            else
            {
                Serial.print("ERROR: Failed to set '");
                Serial.print(settingName);
                Serial.print("' - ");
                if (result == SetResult::INVALID_VALUE)
                {
                    Serial.print("Invalid value. Expected: ");
                    Serial.println(expectedValueType);
                }
                else if (result == SetResult::OUT_OF_RANGE)
                {
                    Serial.print("Value out of range. Expected: ");
                    Serial.println(expectedValueType);
                }
                else if (result == SetResult::UNKNOWN_PARAMETER)
                {
                    Serial.println("Unknown parameter.");
                }
                else
                {
                    Serial.println("Unknown error.");
                }
            }
        }
    }
    else
    {
        if (settings.logging.inCliMode) {
            Serial.print("ERROR: Unknown setting: ");
            Serial.println(settingName);
        }
    }
}

// --- Refactored Command Implementations ---

void CommunicationManager::_displayCliHelp()
{
    if (settings.logging.inCliMode) {
        // --- Helper function to print a formatted line ---
        auto printFormattedLine = [](const String &item, const String &description)
        {
            int itemPadding = 25 - item.length();
            if (itemPadding < 1)
                itemPadding = 1;
            Serial.print("  ");
            Serial.print(item);
            for (int i = 0; i < itemPadding; ++i)
                Serial.print(" ");
            Serial.print("- ");
            Serial.println(description);
        };

        // --- Header ---
        Serial.println("\n--- ESP32 Flight Controller CLI Help ---");
        Serial.println("Provides commands to get, set, and manage flight controller settings.");

        // --- Commands ---
        Serial.println("\n--- Commands ---");
        printFormattedLine("get <param>", "Get the current value of a setting.");
        printFormattedLine("set <param> <value>", "Set a new value for a setting.");
        printFormattedLine("dump", "Display all current settings.");
        printFormattedLine("save", "Save current settings to flash and reboot.");
        printFormattedLine("reset", "Reset all settings to default and reboot.");
        printFormattedLine("reboot", "Reboot the ESP32.");
        printFormattedLine("calibrate_imu", "Start IMU calibration sequence.");
        printFormattedLine("status", "Display system status and metrics.");
        printFormattedLine("version", "Display firmware version.");
        printFormattedLine("help", "Display this help message.");
        printFormattedLine("exit", "Exit CLI mode and return to flight mode.");

        // --- Settings ---
        Serial.println("\n--- Available Settings ---");
        String currentGroup = "";

        for (int i = 0; i < numSettings; ++i)
        {
            const Setting &s = settingsRegistry[i];
            String settingName(s.name);

            // Print group header if it changes
            int dotIndex = settingName.indexOf('.');
            String group = (dotIndex != -1) ? settingName.substring(0, dotIndex) : settingName;
            if (group != currentGroup)
            {
                currentGroup = group;
                String groupHeader = "\n  " + group + " Settings:";
                groupHeader.toUpperCase();
                Serial.println(groupHeader);
            }

            // Prepare description string with type and valid values
            String description = "";
            String expectedValue = "";
            switch (s.type)
            {
            case SettingType::FLOAT:
                description += "[float] ";
                break;
            case SettingType::INT:
                description += "[int]   ";
                break;
            case SettingType::UINT8:
                description += "[uint8] ";
                break;
            case SettingType::UINT16:
                description += "[uint16]";
                break;
            case SettingType::BOOL:
                description += "[bool]  ";
                description += "Values: true, false";
                break;
            case SettingType::ENUM_IBUS_PROTOCOL:
            {
                description += "[enum]  ";
                ReceiverProtocol dummy;
                _parseAndValidateReceiverProtocolSetting("invalid", dummy, expectedValue);
                description += "Values: " + expectedValue;
                break;
            }
            case SettingType::ENUM_IMU_PROTOCOL:
            {
                description += "[enum]  ";
                ImuProtocol dummy;
                _parseAndValidateImuProtocolSetting("invalid", dummy, expectedValue);
                description += "Values: " + expectedValue;
                break;
            }
            case SettingType::ENUM_LPF_BANDWIDTH:
            {
                description += "[enum]  ";
                LpfBandwidth dummy;
                _parseAndValidateLpfBandwidthSetting("invalid", dummy, expectedValue);
                description += "Values: " + expectedValue;
                break;
            }
            case SettingType::ENUM_IMU_ROTATION:
            {
                description += "[enum]  ";
                ImuRotation dummy;
                _parseAndValidateImuRotationSetting("invalid", dummy, expectedValue);
                description += "Values: " + expectedValue;
                break;
            }
            case SettingType::ENUM_DSHOT_MODE:
            {
                description += "[enum]  ";
                dshot_mode_t dummy;
                _parseAndValidateDShotModeSetting("invalid", dummy, expectedValue);
                description += "Values: " + expectedValue;
                break;
            }
            default:
                break;
            }

            printFormattedLine("  " + settingName, description);
        }

        // Manual entry for test string
        Serial.println("\n  Logging Settings (Test String):");
        printFormattedLine("  log.test_string <value>", "[string] Set a test string for logging.");

        // Manual entry for channel mapping
        Serial.println("\n  RX Settings (Channel Mapping):");
        printFormattedLine("  rx.map.<input> <ch>", "[int]   Map a flight control input to a receiver channel (0-15).");
        Serial.println("\n----------------------------------------\n");
    }
}

void CommunicationManager::_displayAllSettingsCliCommand()
{
    if (settings.logging.inCliMode) {
        Serial.println("--- Current Flight Controller Settings ---");
        for (int i = 0; i < numSettings; ++i)
        {
            const Setting &s = settingsRegistry[i];
            Serial.print(s.name);
            Serial.print(": ");
            switch (s.type)
            {
            case SettingType::FLOAT:
                Serial.println(*(float *)s.value / s.scaleFactor, 4);
                break;
            case SettingType::INT:
                if (s.scaleFactor == PID_SCALE_FACTOR)
                {
                    Serial.println(*(int *)s.value / PID_DISPLAY_SCALE_FACTOR);
                }
                else
                {
                    Serial.println(*(int *)s.value);
                }
                break;
            case SettingType::UINT8:
                Serial.println(*(uint8_t *)s.value);
                break;
            case SettingType::UINT16:
                Serial.println(*(uint16_t *)s.value);
                break;
            case SettingType::ULONG:
                Serial.println(*(unsigned long *)s.value);
                break;
            case SettingType::ENUM_IBUS_PROTOCOL:
                Serial.println(_getReceiverProtocolName(*(ReceiverProtocol *)s.value));
                break;
            case SettingType::ENUM_IMU_PROTOCOL:
                Serial.println(_getImuProtocolName(*(ImuProtocol *)s.value));
                break;
            case SettingType::ENUM_LPF_BANDWIDTH:
                Serial.println(_getLpfBandwidthName(*(LpfBandwidth *)s.value));
                break;
            case SettingType::ENUM_IMU_ROTATION:
                Serial.println(_getImuRotationName(*(ImuRotation *)s.value));
                break;
            case SettingType::BOOL:
                Serial.println(_getBoolName(*(bool *)s.value));
                break;
            case SettingType::ENUM_DSHOT_MODE:
                Serial.println(_getDShotModeName(*(dshot_mode_t *)s.value));
                break;
            case SettingType::STRING:
                Serial.println(*(String *)s.value);
                break;
            }
        }
        Serial.println("\n--- Receiver Channel Mapping ---");
        for (int i = 0; i < NUM_FLIGHT_CONTROL_INPUTS; ++i)
        {
            Serial.print("  ");
            Serial.print(_getFlightControlInputName((FlightControlInput)i));
            Serial.print(": ");
            Serial.println(settings.receiver.channelMapping.channel[i]);
        }
        Serial.println("----------------------------------------");
    }
}







void CommunicationManager::_displaySystemStatusCliCommand() const
{
    if (settings.logging.inCliMode) {
        Serial.println("--- System Status ---");
        Serial.print("Target Loop Time (us): ");
        Serial.println(TARGET_LOOP_TIME_US);
        Serial.print("Actual Loop Time (us): ");
        Serial.println(_flightController->state.loopTimeUs);
        Serial.print("CPU Load (%): ");
        Serial.println(_flightController->state.cpuLoad, 2);
        Serial.print("Battery Voltage (V): ");
        Serial.println(_flightController->state.voltage, 2);
        Serial.print("Current Draw (A): ");
        Serial.println(_flightController->state.current, 2);
        Serial.print("Free Heap (bytes): ");
        Serial.println(esp_get_free_heap_size());
        Serial.print("Min Free Heap (bytes): ");
        Serial.println(esp_get_minimum_free_heap_size());
        Serial.println("---------------------");
    }
}

void CommunicationManager::_displayFirmwareVersionCliCommand() const
{
    if (settings.logging.inCliMode) {
        Serial.print("Firmware Version: ");
        Serial.println(FIRMWARE_VERSION);
        // Also print build ID in CLI mode
        Serial.print("Build ID: ");
        Serial.println(FIRMWARE_BUILD_ID);
    }
}