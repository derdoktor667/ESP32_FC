# ESP32 Flight Controller

## Project Overview

This project is a flight controller for a drone or other remote-controlled vehicle, built to run on an ESP32 microcontroller. The main logic is written in C++ using the Arduino framework, now organized into a modular structure within the `src` directory. The primary source file is `ESP32_FC.ino`.

This project uses git submodules to manage external libraries.

## Features

*   **MPU6050 IMU Integration**: Reads accelerometer and gyroscope data for flight stabilization.
*   **Flysky i-BUS Receiver Support**: Decodes signals from Flysky transmitters for control input.
*   **DShot ESC Control**: Controls Electronic Speed Controllers using the DShot300 protocol for precise motor control.
*   **Attitude Estimation**: Implemented a complementary filter to combine accelerometer and gyroscope data for stable roll, pitch, and yaw estimation.
*   **PID Controller**: Basic PID controller structure for roll, pitch, and yaw control.
*   **Motor Mixing**: Logic to mix throttle and PID outputs for individual motor control in an X-quad configuration.
*   **MPU6050 Calibration**: Gyroscope and accelerometer calibration routines to improve sensor accuracy.
*   **Arming/Disarming**: Safety mechanism to arm and disarm motors based on a transmitter switch.
*   **Enhanced Serial Output**: Consolidated and interval-controlled serial output for flight status information.
*   **Code Readability**: Improved code readability and maintainability by replacing magic numbers with named constants.

## Getting Started

To build and upload the code to an ESP32, you will need the [Arduino IDE](https://www.arduino.cc/en/software) or the [Arduino CLI](https://arduino.github.io/arduino-cli/latest/).

### Initial Setup

After cloning the repository, you must initialize and update the git submodules:

```bash
git submodule update --init --recursive
```
## Libraries and Submodules

This project uses the following libraries, managed as git submodules in the `libraries` directory:

*   **`DShotRMT`**: A library to control Electronic Speed Controllers (ESCs) using the DShot protocol. It appears to be specifically adapted for the ESP32's RMT (Remote Control) peripheral.
*   **`ESP32_MPU6050`**: A library for interfacing with the MPU6050 IMU (Inertial Measurement Unit) on the ESP32.
*   **`FlyskyIBUS`**: A library for decoding the FlySky i-BUS protocol, used for communication between the radio receiver and the flight controller.

## Development Conventions

The code follows standard Arduino conventions:

*   The main entry points are the `setup()` and `loop()` functions.
*   `setup()` is used for initialization.
*   `loop()` contains the main, repeating logic of the flight controller.

## Code Structure

The firmware has been modularized into a `src` directory for better organization and maintainability. The main `ESP32_FC.ino` file now acts as an orchestrator, including various modules:

*   `config.h`: Contains all global constants and pin definitions.
*   `pid_controller.h/.cpp`: Implements the PIDController struct and its logic.
*   `attitude_estimator.h/.cpp`: Handles attitude estimation using the complementary filter.
*   `arming_disarming.h/.cpp`: Manages the arming and disarming logic.
*   `flight_modes.h/.cpp`: Implements flight mode selection and related logic.
*   `mpu_calibration.h/.cpp`: Contains MPU6050 gyroscope and accelerometer calibration routines.
*   `serial_logger.h/.cpp`: Manages serial output for flight status information.
*   `motor_control.h/.cpp`: Handles motor initialization, mixing, and DShot command sending.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
