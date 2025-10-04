#include <Arduino.h>
#include <ESP32_MPU6050.h>
#include <FlyskyIBUS.h>
#include <DShotRMT.h>

#include "src/config.h"
#include "src/pid_controller.h"
#include "src/attitude_estimator.h"
#include "src/arming_disarming.h"
#include "src/flight_modes.h"
#include "src/mpu_calibration.h"
#include "src/serial_logger.h"
#include "src/motor_control.h"

// Global objects
ESP32_MPU6050 imuSensor;
FlyskyIBUS ibusReceiver(Serial2, IBUS_RX_PIN);

// Global variables
PIDController pid_roll(settings.pidRoll.kp, settings.pidRoll.ki, settings.pidRoll.kd);
PIDController pid_pitch(settings.pidPitch.kp, settings.pidPitch.ki, settings.pidPitch.kd);
PIDController pid_yaw(settings.pidYaw.kp, settings.pidYaw.ki, settings.pidYaw.kd);

float target_roll = 0.0;
float target_pitch = 0.0;
float target_yaw = 0.0;

// Global variables
extern float roll, pitch, yaw;
extern unsigned long last_attitude_update_time;
extern float gyro_offset_x, gyro_offset_y, gyro_offset_z;
extern float acc_offset_x, acc_offset_y, acc_offset_z;
extern bool armed;
extern bool failsafeActive;
extern int arming_channel_value;
extern FlightMode current_flight_mode;
extern unsigned long last_print_time;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

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

void loop()
{
  imuSensor.update();
  calculateAttitude();
  handleSafetySwitches();
  handleFlightModeSelection();

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

  float pid_output_roll = pid_roll.calculate(target_roll, roll);
  float pid_output_pitch = pid_pitch.calculate(target_pitch, pitch);
  float pid_output_yaw = pid_yaw.calculate(target_yaw, yaw);

  static int ibus_throttle = 0;
  static int throttle = 0;

  ibus_throttle = ibusReceiver.getChannel(IBUS_CHANNEL_THROTTLE);
  throttle = map(ibus_throttle, settings.receiver.ibusMinValue, settings.receiver.ibusMaxValue, settings.dshotThrottle.min, settings.dshotThrottle.max);

  sendMotorCommands(throttle, pid_output_roll, pid_output_pitch, pid_output_yaw, armed);

  if (millis() - last_print_time >= settings.printIntervalMs)
  {
    printFlightStatus();
    last_print_time = millis();
  }
}
