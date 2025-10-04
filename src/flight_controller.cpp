#include "flight_controller.h"
#include <Arduino.h>

// Include all necessary modules
#include "config.h"
#include "settings.h"
#include "pid_controller.h"
#include "attitude_estimator.h"
#include "arming_disarming.h"
#include "flight_modes.h"
#include "mpu_calibration.h"
#include "serial_logger.h"
#include "motor_control.h"
#include "cli.h"

// --- Global Objects and Variables ---
// These were moved from ESP32_FC.ino

// Hardware and PID controllers
ESP32_MPU6050 imuSensor;
FlyskyIBUS ibusReceiver(Serial2, IBUS_RX_PIN);
PIDController pid_roll(settings.pidRoll.kp, settings.pidRoll.ki, settings.pidRoll.kd);
PIDController pid_pitch(settings.pidPitch.kp, settings.pidPitch.ki, settings.pidPitch.kd);
PIDController pid_yaw(settings.pidYaw.kp, settings.pidYaw.ki, settings.pidYaw.kd);

// Motor objects
DShotRMT motorFrontRight(ESC_PIN_1, DSHOT300, false);
DShotRMT motorFrontLeft(ESC_PIN_2, DSHOT300, false);
DShotRMT motorRearRight(ESC_PIN_3, DSHOT300, false);
DShotRMT motorRearLeft(ESC_PIN_4, DSHOT300, false);

// Target setpoints
float target_roll = 0.0;
float target_pitch = 0.0;
float target_yaw = 0.0;

// --- Function Implementations ---

void initializeFlightController()
{
    loadSettings(); // Load settings from flash memory

    Serial.println("Initializing MPU6050...");
    if (!imuSensor.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 initialized successfully!");

    calibrateImuSensor();

    Serial.println("Initializing FlyskyIBUS...");
    ibusReceiver.begin();
    Serial.println("FlyskyIBUS initialized successfully!");

    setupMotors();

    last_attitude_update_time = micros();
}

void runFlightLoop()
{
    // --- Core Flight Logic ---
    imuSensor.update();
    calculateAttitude();
    handleSafetySwitches();
    handleFlightModeSelection();

    // --- Setpoint Calculation ---
    if (current_flight_mode == ANGLE_MODE)
    {
        target_roll = map(ibusReceiver.getChannel(IBUS_CHANNEL_ROLL), settings.receiver.ibusMinValue, settings.receiver.ibusMaxValue, -settings.rates.targetAngleRollPitch, settings.rates.targetAngleRollPitch);
        target_pitch = map(ibusReceiver.getChannel(IBUS_CHANNEL_PITCH), settings.receiver.ibusMinValue, settings.receiver.ibusMaxValue, -settings.rates.targetAngleRollPitch, settings.rates.targetAngleRollPitch);
        target_yaw = map(ibusReceiver.getChannel(IBUS_CHANNEL_YAW), settings.receiver.ibusMinValue, settings.receiver.ibusMaxValue, -settings.rates.targetRateYaw, settings.rates.targetRateYaw);
    }
    else
    { // ACRO_MODE
        target_roll = map(ibusReceiver.getChannel(IBUS_CHANNEL_ROLL), settings.receiver.ibusMinValue, settings.receiver.ibusMaxValue, -settings.rates.targetRateRollPitch, settings.rates.targetRateRollPitch);
        target_pitch = map(ibusReceiver.getChannel(IBUS_CHANNEL_PITCH), settings.receiver.ibusMinValue, settings.receiver.ibusMaxValue, -settings.rates.targetRateRollPitch, settings.rates.targetRateRollPitch);
        target_yaw = map(ibusReceiver.getChannel(IBUS_CHANNEL_YAW), settings.receiver.ibusMinValue, settings.receiver.ibusMaxValue, -settings.rates.targetRateYaw, settings.rates.targetRateYaw);
    }

    // --- PID Calculation ---
    float pid_output_roll = pid_roll.calculate(target_roll, roll);
    float pid_output_pitch = pid_pitch.calculate(target_pitch, pitch);
    float pid_output_yaw = pid_yaw.calculate(target_yaw, yaw);

    // --- Motor Command ---
    static int ibus_throttle = 0;
    ibus_throttle = ibusReceiver.getChannel(IBUS_CHANNEL_THROTTLE);
    int throttle = map(ibus_throttle, settings.receiver.ibusMinValue, settings.receiver.ibusMaxValue, settings.dshotThrottle.min, settings.dshotThrottle.max);

    sendMotorCommands(throttle, pid_output_roll, pid_output_pitch, pid_output_yaw, armed);

    // --- Logging and CLI ---
    if (millis() - last_print_time >= settings.printIntervalMs)
    {
        printFlightStatus();
        last_print_time = millis();
    }

    handleSerialCli();
}
