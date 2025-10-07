#include "Mpu6050Imu.h"

// Constructor: Initializes the MPU6050 object with the given Wire port.
Mpu6050Imu::Mpu6050Imu(TwoWire *wirePort)
    : _mpu() // Initialize _mpu with default constructor
{
    // Store wirePort for later use in begin()
    // The ESP32_MPU6050 library's begin() method takes the TwoWire* argument.
}

// Initializes the MPU6050 sensor.
bool Mpu6050Imu::begin()
{
    // Pass the wirePort to the MPU6050's begin method
    return _mpu.begin(GYRO_RANGE_2000DPS, ACCEL_RANGE_16G); // Highest ranges
}

// Updates the raw sensor data from the MPU6050.
void Mpu6050Imu::update()
{
    _mpu.update();
}

// Performs MPU6050-specific calibration.
bool Mpu6050Imu::calibrate(int numReadings)
{
    Serial.println("Calibrating MPU6050... Keep the drone still.");
    _mpu.calibrate(numReadings);
    Serial.println("MPU6050 Calibration complete.");
    return true; // MPU6050 library calibration always returns true if it runs
}
