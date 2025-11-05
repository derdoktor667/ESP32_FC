# ESP32 Flight Controller Pinout

This file documents the hardware pin connections for the ESP32-based flight controller.

## MCU: ESP32

| Function            | Pin Name          | ESP32 Pin |
| ------------------- | ----------------- | --------- |
| **IMU (MPU6050)**   |                   |           |
| I2C Clock           | SCL               | GPIO 22   |
| I2C Data            | SDA               | GPIO 21   |
| **ESCs (DShot)**    |                   |           |
| Motor 1 (Front-Right) | ESC_PIN_FRONT_RIGHT | GPIO 27   |
| Motor 2 (Front-Left)  | ESC_PIN_FRONT_LEFT  | GPIO 25   |
| Motor 3 (Rear-Right)  | ESC_PIN_REAR_RIGHT  | GPIO 26   |
| Motor 4 (Rear-Left)   | ESC_PIN_REAR_LEFT   | GPIO 33   |
| **Receiver**        |                   |           |
| IBUS/PPM Signal     | RECEIVER_RX_PIN   | GPIO 16   |
