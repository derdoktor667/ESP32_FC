# 🚁 ESP32 Flight Controller

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/derdoktor667/ESP32_FC/ci.yml?branch=main&style=for-the-badge) ![GitHub](https://img.shields.io/github/license/derdoktor667/ESP32_FC?style=for-the-badge)

An advanced flight controller firmware for quadcopters and other RC vehicles, built on the powerful ESP32 platform and the Arduino framework.

<p align="center">
  <img src="libraries/DShotRMT/img/dshot300.png" alt="DShot Signal Diagram" width="600"/>
</p>

---

## ✨ Key Features

*   **Robust Flight Stabilization**: Utilizes an MPU6050 IMU combined with a complementary filter for precise attitude estimation (Roll, Pitch, Yaw).
*   **High-Performance Motor Control**: Employs hardware-accelerated DShot300 for low-latency, high-resolution communication with ESCs.
*   **Flexible Control System**: 
    *   Supports Flysky i-BUS protocol for remote control.
    *   Features multiple flight modes (e.g., Angle, Acro).
    *   Includes a critical **Failsafe Switch** to immediately cut motor power.
*   **Centralized Configuration**: All tunable parameters (PIDs, rates, filters) are neatly organized in a single `FlightControllerSettings` struct for easy adjustments.
*   **Modular & Maintainable Code**: The source code is logically split into modules, following the Single Responsibility Principle for better readability and maintenance.
*   **Safety First**: A dedicated arming/disarming mechanism prevents accidental motor startup.

---

## 🚀 Getting Started

Follow these steps to get the flight controller up and running on your ESP32.

### Prerequisites

*   An ESP32 development board.
*   [Arduino IDE](https://www.arduino.cc/en/software) or [Arduino CLI](https://arduino.github.io/arduino-cli/latest/) installed.
*   All necessary hardware (MPU6050, ESCs, motors, Flysky receiver) connected according to the pin definitions in `src/config.h`.

### Installation

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/derdoktor667/ESP32_FC.git
    cd ESP32_FC
    ```

2.  **Initialize Git Submodules:** This project relies on external libraries managed as submodules. Pull them in with this command:
    ```bash
    git submodule update --init --recursive
    ```

3.  **Compile and Upload:** Open the `ESP32_FC.ino` sketch in your Arduino IDE, select your ESP32 board, and upload. Alternatively, use the Arduino CLI:
    ```bash
    arduino-cli compile --fqbn esp32:esp32:esp32 ESP32_FC.ino
    arduino-cli upload --port /dev/ttyUSB0 --fqbn esp32:esp32:esp32 ESP32_FC.ino
    ```
    *(Replace `/dev/ttyUSB0` with the actual port of your ESP32.)*

---

## 🛠️ Code Structure

The firmware is organized into a clean, modular structure within the `src/` directory. The main `ESP32_FC.ino` file acts as the orchestrator.

| Module                  | Responsibility                                                                  |
| ----------------------- | ------------------------------------------------------------------------------- |
| `config.h`              | Central hub for hardware pins and the `FlightControllerSettings` struct.        |
| `pid_controller.h/.cpp`   | Implements the PID control loop for stabilization.                              |
| `attitude_estimator.h/.cpp` | Calculates the drone's orientation using sensor fusion.                         |
| `arming_disarming.h/.cpp` | Manages all safety logic: Arm, Disarm, and the high-priority Failsafe.          |
| `flight_modes.h/.cpp`     | Handles switching between different flight modes (Angle, Acro, etc.).           |
| `mpu_calibration.h/.cpp`  | Contains routines to calibrate the MPU6050 sensor on startup.                   |
| `serial_logger.h/.cpp`    | Manages the formatted serial output for debugging and status monitoring.        |
| `motor_control.h/.cpp`    | Handles motor layout, mixing, and sending commands via DShot.                 |

---

## 📚 Libraries & Submodules

This project stands on the shoulders of giants. The core hardware interaction is handled by these libraries, included as Git submodules:

*   [**DShotRMT**](libraries/DShotRMT): A fantastic library for generating DShot signals using the ESP32's RMT peripheral.
*   [**ESP32_MPU6050**](libraries/ESP32_MPU6050): A driver for the MPU6050 IMU.
*   [**FlyskyIBUS**](libraries/FlyskyIBUS): A lightweight library for decoding the Flysky i-BUS protocol.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for full details.