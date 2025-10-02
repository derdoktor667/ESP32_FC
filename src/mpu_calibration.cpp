#include "mpu_calibration.h"

// External MPU object
extern ESP32_MPU6050 mpu;

// Gyroscope offsets
float gyro_offset_x = 0.0;
float gyro_offset_y = 0.0;
float gyro_offset_z = 0.0;

// Accelerometer offsets
float acc_offset_x = 0.0;
float acc_offset_y = 0.0;
float acc_offset_z = 0.0;

void calibrateMPU6050()
{
  Serial.println("Calibrating MPU6050 Gyroscope and Accelerometer...");
  int num_readings = MPU_CALIBRATION_READINGS;
  float temp_gyro_x = 0, temp_gyro_y = 0, temp_gyro_z = 0;
  float temp_acc_x = 0, temp_acc_y = 0, temp_acc_z = 0;

  for (int i = 0; i < num_readings; i++)
  {
    mpu.update();
    temp_gyro_x += mpu.readings.gyroscope.x;
    temp_gyro_y += mpu.readings.gyroscope.y;
    temp_gyro_z += mpu.readings.gyroscope.z;
    temp_acc_x += mpu.readings.accelerometer.x;
    temp_acc_y += mpu.readings.accelerometer.y;
    temp_acc_z += mpu.readings.accelerometer.z;
    delay(1); // Small delay to allow new readings
  }

  gyro_offset_x = temp_gyro_x / num_readings;
  gyro_offset_y = temp_gyro_y / num_readings;
  gyro_offset_z = temp_gyro_z / num_readings;

  acc_offset_x = temp_acc_x / num_readings;
  acc_offset_y = temp_acc_y / num_readings;
  acc_offset_z = (temp_acc_z / num_readings) - ACCEL_Z_GRAVITY; // Assuming Z-axis is up and should read 1g

  Serial.print("GyroX Offset: ");
  Serial.println(gyro_offset_x);
  Serial.print("GyroY Offset: ");
  Serial.println(gyro_offset_y);
  Serial.print("GyroZ Offset: ");
  Serial.println(gyro_offset_z);
  Serial.print("AccX Offset: ");
  Serial.println(acc_offset_x);
  Serial.print("AccY Offset: ");
  Serial.println(acc_offset_y);
  Serial.print("AccZ Offset: ");
  Serial.println(acc_offset_z);
  Serial.println("MPU6050 Gyroscope and Accelerometer calibration complete.");
}
