#include "mpu_calibration.h"

// External MPU object
extern ESP32_MPU6050 imuSensor;

// Gyroscope offsets
float gyro_offset_x = 0.0;
float gyro_offset_y = 0.0;
float gyro_offset_z = 0.0;

// Accelerometer offsets
float acc_offset_x = 0.0;
float acc_offset_y = 0.0;
float acc_offset_z = 0.0;

void calibrateImuSensor()
{
  Serial.println("Calibrating MPU6050 Gyroscope and Accelerometer...");
  int num_readings = settings.calibration.mpuCalibrationReadings;
  float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
  float accelSumX = 0, accelSumY = 0, accelSumZ = 0;

  for (int i = 0; i < num_readings; i++)
  {
    imuSensor.update();
    gyroSumX += imuSensor.readings.gyroscope.x;
    gyroSumY += imuSensor.readings.gyroscope.y;
    gyroSumZ += imuSensor.readings.gyroscope.z;
    accelSumX += imuSensor.readings.accelerometer.x;
    accelSumY += imuSensor.readings.accelerometer.y;
    accelSumZ += imuSensor.readings.accelerometer.z;
    delay(1); // Small delay to allow new readings
  }

  gyro_offset_x = gyroSumX / num_readings;
  gyro_offset_y = gyroSumY / num_readings;
  gyro_offset_z = gyroSumZ / num_readings;

  acc_offset_x = accelSumX / num_readings;
  acc_offset_y = accelSumY / num_readings;
  acc_offset_z = (accelSumZ / num_readings) - settings.calibration.accelZGravity; // Assuming Z-axis is up and should read 1g

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
