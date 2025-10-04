#include "attitude_estimator.h"
#include "flight_controller.h"

// Define external attitude variables
float roll = 0.0, pitch = 0.0, yaw = 0.0;
unsigned long last_attitude_update_time = 0;

void calculateAttitude()
{
  unsigned long current_time = micros();
  float dt = (current_time - last_attitude_update_time) / 1000000.0; // Convert to seconds
  last_attitude_update_time = current_time;

  // Read raw gyroscope and accelerometer data
  float gyroX = imuSensor.readings.gyroscope.x - gyro_offset_x;
  float gyroY = imuSensor.readings.gyroscope.y - gyro_offset_y;
  float gyroZ = imuSensor.readings.gyroscope.z - gyro_offset_z;
  float accX = imuSensor.readings.accelerometer.x - acc_offset_x;
  float accY = imuSensor.readings.accelerometer.y - acc_offset_y;
  float accZ = imuSensor.readings.accelerometer.z - acc_offset_z;

  // Convert accelerometer readings to roll and pitch angles
  // Assuming the MPU6050 is mounted flat
  float acc_roll = atan2(accY, accZ) * 180 / PI;
  float acc_pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // Apply complementary filter
  // Gyroscope provides short-term accuracy, accelerometer provides long-term stability
  roll = settings.filter.complementaryFilterGain * (roll + gyroX * dt) + (1.0 - settings.filter.complementaryFilterGain) * acc_roll;
  pitch = settings.filter.complementaryFilterGain * (pitch + gyroY * dt) + (1.0 - settings.filter.complementaryFilterGain) * acc_pitch;
  yaw = yaw + gyroZ * dt; // Yaw is integrated directly from gyroscope, as accelerometer cannot provide yaw reference
}
