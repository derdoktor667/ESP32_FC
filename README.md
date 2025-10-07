# 🚁 ESP32 Flight Controller

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/derdoktor667/ESP32_FC/ci.yml?branch=main&style=for-the-badge) ![GitHub](https://img.shields.io/github/license/derdoktor667/ESP32_FC?style=for-the-badge)

An advanced flight controller firmware for quadcopters and other RC vehicles, built on the powerful ESP32 platform and the Arduino framework.

---

## ✨ Key Features

*   **Modular Object-Oriented Architecture**: The entire flight control logic is now encapsulated in dedicated C++ classes, promoting clear responsibilities, explicit data flow, and easy extensibility.
*   **Centralized Flight State**: A single `FlightState` struct holds all dynamic data (attitude, setpoints, armed status, etc.), making the drone's current state transparent and easy to manage.
*   **Robust Flight Stabilization**: Utilizes an MPU6050 IMU combined with a complementary filter for precise attitude estimation (Roll, Pitch, Yaw), now managed by the `AttitudeEstimator` module.
*   **High-Performance Motor Control**: Employs hardware-accelerated DShot300 for low-latency, high-resolution communication with ESCs, orchestrated by the `MotorMixer` module.
*   **Flexible Control System**: Supports interchangeable receiver protocols (initially i-BUS and custom PPM) thanks to a clean abstraction layer and `SetpointManager` for control input processing.
*   **Critical Safety Management**: Dedicated `SafetyManager` module handles arming/disarming and high-priority Failsafe logic.
*   **Centralized Configuration**: All tunable parameters (PIDs, rates, filters) are neatly organized in a single `FlightControllerSettings` struct for easy adjustments.
*   **Interactive CLI Tuning**: A user-friendly serial CLI allows for on-the-fly adjustments of all settings, including PIDs, rates, and sensor calibration, without needing to recompile.

---

## 🚀 Getting Started

Follow these steps to get the flight controller up and running on your ESP32.

### Prerequisites

*   An ESP32 development board.
*   [Arduino IDE](https://www.arduino.cc/en/software) or [Arduino CLI](https://arduino.github.io/arduino-cli/latest/) installed.
*   All necessary hardware (MPU6050, ESCs, motors, RC receiver) connected according to the pin definitions in `src/config.h`.

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

## 💻 CLI Usage

The ESP32 Flight Controller features an interactive Command-Line Interface (CLI) accessible via the serial monitor. This allows for real-time monitoring, configuration adjustments, and triggering of specific actions without recompiling the firmware.

### Activating the CLI

1.  Connect your ESP32 to your computer and open a serial monitor (e.g., Arduino IDE Serial Monitor, PuTTY, minicom) at a baud rate of `115200`.
2.  Type `cli` and press Enter.
3.  The CLI will activate, display a welcome header, and present the `ESP32_FC >` prompt.

### Available Commands

Once activated, you can use the following commands:

*   `get <parameter>`: Retrieves the current value of a specific setting.
    *   **Example:** `get pid.roll.kp`
    *   **Example:** `get rates.angle`
*   `set <parameter> <value>`: Sets a new value for a specific setting.
    *   **Example:** `set pid.roll.kp 0.95`
    *   **Example:** `set rates.angle 25.0`
*   `dump`: Prints all current flight controller settings in a structured, human-readable format.
*   `debug`: Prints a snapshot of real-time flight data (IMU readings, attitude, setpoints, arming status, etc.).
*   `save`: Saves the current settings to the ESP32's non-volatile memory (flash). Settings are persistent across reboots.
*   `reset`: Resets all settings to their default values and saves them to flash.
*   `reboot`: Reboots the ESP32 flight controller.
*   `calibrate_imu`: Triggers a manual calibration of the MPU6050 IMU sensor. Ensure the drone is level and still during calibration.
*   `help`: Displays a list of all available commands and their descriptions.
*   `exit`: Deactivates the CLI and returns to normal serial logging (if enabled).

### Example Session

```
--- ESP32_FC CLI Activated ---
========================================
    ______ _____ ____    _______  __
   / ____// ___// __ \\  / ____/ |/ /
  / __/   \\__ \\/ / / / /_   |   / 
 / /___  ___/ / /_/ / / __/  /   |  
/_____/ /____/\\____/ /_/    /_/|_|  
         Flight Controller CLI         
========================================

Available commands:
  get <parameter>       - Get a specific setting value.
  set <param> <value>   - Set a new value for a setting.
  dump                  - Print all current settings.
  debug                 - Print a snapshot of runtime flight data.
  save                  - Save current settings to flash memory.
  reset                 - Reset all settings to defaults.
  reboot                - Reboot the flight controller.
  calibrate_imu         - Manually trigger IMU calibration.
  help                  - Show this help message.
  exit                  - Deactivate the CLI.

ESP32_FC > get pid.roll.kp
0.8000
ESP32_FC > set pid.roll.kp 0.9
Set pid.roll.kp to 0.9
ESP32_FC > dump

--- [ Flight Controller Settings ] ---

--- PID Settings ---
  pid.roll.kp              : 0.9000
  pid.roll.ki              : 0.0010
  pid.roll.kd              : 0.0500

  pid.pitch.kp             : 0.8000
  pid.pitch.ki             : 0.0010
  pid.pitch.kd             : 0.0500

  pid.yaw.kp               : 1.5000
  pid.yaw.ki               : 0.0050
  pid.yaw.kd               : 0.1000

  pid.integral_limit       : 400.00

--- Rate Settings ---
  rates.angle              : 30.00 (deg)
  rates.yaw                : 90.00 (deg/s)
  rates.acro               : 90.00 (deg/s)

--- Filter Settings ---
  filter.gain              : 0.98

--- Receiver Settings ---
  rx.min                   : 1000
  rx.max                   : 2000
  rx.arming_threshold      : 1500
  rx.failsafe_threshold    : 1500

--- Logging Settings ---
  log.interval             : 100 (ms)

--------------------------------------
ESP32_FC > save
Settings saved to flash memory.
ESP32_FC > exit
```

---

## 🛠️ Code Structure

The firmware is organized into a clean, modular, object-oriented structure within the `src/` directory.

| Module                    | Responsibility                                                                  |
| ------------------------- | ------------------------------------------------------------------------------- |
| `FlightState.h`           | Defines the central `FlightState` struct, encapsulating all dynamic flight data. |
| `flight_controller.h/.cpp`  | The main `FlightController` class, orchestrating all modules and the main loop. |
| `config.h`                | Central hub for hardware definitions, `FlightControllerSettings` struct, and `FlightMode` enum. |
| `settings.h/.cpp`         | Manages loading and saving settings to persistent storage.                      |
| `ReceiverInterface.h`     | Defines the abstract base class for all RC receiver protocols.                  |
| `IbusReceiver.h/.cpp`     | Concrete implementation for the Flysky i-BUS protocol.                        |
| `PpmReceiver.h/.cpp`      | Custom implementation for the PPM protocol.                                   |
| `pid_controller.h/.cpp`     | PID controller logic (used by `PidProcessor`).                                  |
| `serial_logger.h/.cpp`      | Manages formatted serial output for debugging and status monitoring.            |
| `cli.h/.cpp`              | User-friendly serial command-line interface for runtime configuration.          |
| `modules/`                | Directory containing the core processing modules:                               |
| &nbsp;&nbsp;`AttitudeEstimator.h/.cpp` | Encapsulates IMU reading, calibration, and complementary filter for attitude estimation. |
| &nbsp;&nbsp;`SafetyManager.h/.cpp`     | Manages arming, disarming, and failsafe logic based on receiver input.          |
| &nbsp;&nbsp;`SetpointManager.h/.cpp`   | Calculates target setpoints for roll, pitch, and yaw based on receiver input and flight mode. |
| &nbsp;&nbsp;`PidProcessor.h/.cpp`      | Manages and executes the PID control loops for all axes.                        |
| &nbsp;&nbsp;`MotorMixer.h/.cpp`        | Mixes PID outputs with throttle and sends commands to the motors.               |
---

## 📚 Libraries & Submodules

This project stands on the shoulders of giants. The core hardware interaction is handled by these libraries, included as Git submodules:

*   [**DShotRMT**](libraries/DShotRMT): A fantastic library for generating DShot signals using the ESP32's RMT peripheral.
*   [**ESP32_MPU6050**](libraries/ESP32_MPU6050): A driver for the MPU6050 IMU.
*   [**FlyskyIBUS**](libraries/FlyskyIBUS): A lightweight library for decoding the Flysky i-BUS protocol.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for full details.
