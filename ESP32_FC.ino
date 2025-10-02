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
ESP32_MPU6050 mpu;
FlyskyIBUS ibus(Serial2, IBUS_RX_PIN);

// Global variables (defined here, declared extern in their respective headers)
PIDController pid_roll(0.8, 0.001, 0.05);  // Kp, Ki, Kd - these values will need careful tuning!
PIDController pid_pitch(0.8, 0.001, 0.05); // Kp, Ki, Kd - these values will need careful tuning!
PIDController pid_yaw(1.5, 0.005, 0.1);    // Kp, Ki, Kd - these values will need careful tuning!

float target_roll = 0.0;
float target_pitch = 0.0;
float target_yaw = 0.0;

// Global variables (declared extern in their respective headers and defined in their .cpp files)
extern float roll, pitch, yaw;
extern unsigned long last_attitude_update_time;
extern float gyro_offset_x, gyro_offset_y, gyro_offset_z;
extern float acc_offset_x, acc_offset_y, acc_offset_z;
extern bool armed;
extern int arming_channel_value;
extern FlightMode current_flight_mode;
extern unsigned long last_print_time;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 initialized successfully!");

  calibrateMPU6050();

  Serial.println("Initializing FlyskyIBUS...");
  ibus.begin();
  Serial.println("FlyskyIBUS initialized successfully!");

  setupMotors();

  last_attitude_update_time = micros();
}

void loop()
{
  mpu.update();
  calculateAttitude();
  handleArming();
  handleFlightModeSelection();

  if (current_flight_mode == ANGLE_MODE)
  {
    target_roll = map(ibus.getChannel(IBUS_CHANNEL_ROLL), IBUS_MIN_VALUE, IBUS_MAX_VALUE, -TARGET_ANGLE_ROLL_PITCH, TARGET_ANGLE_ROLL_PITCH);
    target_pitch = map(ibus.getChannel(IBUS_CHANNEL_PITCH), IBUS_MIN_VALUE, IBUS_MAX_VALUE, -TARGET_ANGLE_ROLL_PITCH, TARGET_ANGLE_ROLL_PITCH);
    target_yaw = map(ibus.getChannel(IBUS_CHANNEL_YAW), IBUS_MIN_VALUE, IBUS_MAX_VALUE, -TARGET_ANGLE_YAW_RATE, TARGET_ANGLE_YAW_RATE);
  }
  else
  { // ACRO_MODE
    target_roll = map(ibus.getChannel(IBUS_CHANNEL_ROLL), IBUS_MIN_VALUE, IBUS_MAX_VALUE, -TARGET_ANGLE_YAW_RATE, TARGET_ANGLE_YAW_RATE);
    target_pitch = map(ibus.getChannel(IBUS_CHANNEL_PITCH), IBUS_MIN_VALUE, IBUS_MAX_VALUE, -TARGET_ANGLE_YAW_RATE, TARGET_ANGLE_YAW_RATE);
    target_yaw = map(ibus.getChannel(IBUS_CHANNEL_YAW), IBUS_MIN_VALUE, IBUS_MAX_VALUE, -TARGET_ANGLE_YAW_RATE, TARGET_ANGLE_YAW_RATE);
  }

  float pid_output_roll = pid_roll.calculate(target_roll, roll);
  float pid_output_pitch = pid_pitch.calculate(target_pitch, pitch);
  float pid_output_yaw = pid_yaw.calculate(target_yaw, yaw);

  static int ibus_throttle = 0;
  static int throttle = 0;

  ibus_throttle = ibus.getChannel(IBUS_CHANNEL_THROTTLE);
  throttle = map(ibus_throttle, IBUS_MIN_VALUE, IBUS_MAX_VALUE, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);

  sendMotorCommands(throttle, pid_output_roll, pid_output_pitch, pid_output_yaw, armed);

  if (millis() - last_print_time >= PRINT_INTERVAL_MS)
  {
    printFlightStatus();
    last_print_time = millis();
  }
}
