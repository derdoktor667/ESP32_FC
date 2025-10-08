# 🚁 ESP32 Flight Controller

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/derdoktor667/ESP32_FC/ci.yml?branch=main&style=for-the-badge) ![GitHub](https://img.shields.io/github/license/derdoktor667/ESP32_FC?style=for-the-badge)

An advanced flight controller firmware for quadcopters and other RC vehicles, built on the powerful ESP32 platform and the Arduino framework.

---

## ✨ Key Features

*   **Modular Object-Oriented Architecture**: The entire flight control logic is now encapsulated in dedicated C++ classes, promoting clear responsibilities, explicit data flow, and easy extensibility.
*   **Centralized Flight State**: A single `FlightState` struct holds all dynamic data (attitude, setpoints, armed status, etc.), making the drone's current state transparent and easy to manage.
*   **Robust Flight Stabilization**: Utilizes an `ImuInterface` abstraction for flexible IMU sensor support (initially MPU6050, defaulting to highest range settings) combined with a **Madgwick filter** for precise attitude estimation (Roll, Pitch, Yaw), now managed by the `AttitudeEstimator` module.
*   **High-Performance Motor Control**: Employs hardware-accelerated DShot300 for low-latency, high-resolution communication with ESCs, orchestrated by the `MotorMixer` module.
*   **Flexible Control System**: Supports interchangeable receiver protocols (initially i-BUS and custom PPM) and features **configurable channel mapping** for assigning receiver channels to flight control inputs (Throttle, Roll, Pitch, Yaw, Arm, Failsafe, Flight Mode) thanks to a clean abstraction layer and `SetpointManager` for control input processing.
*   **Critical Safety Management**: Dedicated `SafetyManager` module handles arming/disarming and high-priority Failsafe logic, now including both receiver signal loss detection and a configurable failsafe switch.
*   **Centralized Configuration**: All tunable parameters (PIDs, rates, filters, motor idle speed) are neatly organized in a single `FlightControllerSettings` struct for easy adjustments. PID gains are stored as scaled integers, allowing for fine adjustment via the CLI.
*   **Interactive CLI Tuning**: A user-friendly serial CLI allows for on-the-fly adjustments of all settings, including PIDs (with fine-grained control), rates, sensor calibration, motor idle speed, and IMU/receiver protocol selection. Settings are saved to flash and the ESP32 automatically reboots to apply changes.
*   **Persistent Serial Logging**: On-demand, interval-based serial logging is controllable via CLI (`log on/off` commands). The global `enableLogging` flag and `printIntervalMs` (defaulting to 0 for disabled logging) are persistent settings, ensuring logging preferences are retained across reboots.
*   **Web-based Configurator (`/webui`)**: A modern, visually appealing web application built with plain HTML, CSS, and JavaScript for convenient PID tuning and CLI interaction via the Web Serial API. Features include: 
    *   Connect/Disconnect functionality.
    *   Real-time PID adjustment for Roll, Pitch, and Yaw.
    *   Direct CLI command input.
    *   On-demand live data display with a toggle switch.
    *   Dark theme for enhanced user experience.

---

## ⚙️ Development Environment / Compatibility

This project is developed for and tested with:

*   **Target Platform**: ESP32
*   **ESP-IDF Version**: v5.5 (specifically v5.5.1) - [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/)
*   **Framework**: Arduino for ESP32
*   **Development IDE**: Visual Studio Code
*   **Operating System**: Arch Linux

---

## 🚀 Getting Started

Follow these steps to get the flight controller up and running on your ESP32.

### Prerequisites

*   An ESP32 development board.
*   [Arduino IDE](https://www.arduino.cc/en/software) or [Arduino CLI](https://arduino.github.io/arduino-cli/latest/) installed.
*   All necessary hardware (MPU6050, ESCs, motors, RC receiver) connected according to the pin definitions in `src/config.h`.
*   A modern web browser supporting the [Web Serial API](https://developer.chrome.com/docs/capabilities/web-apis/web-serial) (e.g., Google Chrome) for the web configurator.

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

3.  **Compile and Upload Firmware:** Open the `ESP32_FC.ino` sketch in your Arduino IDE, select your ESP32 board, and upload. Alternatively, use the Arduino CLI:
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
*   `debug`: Prints a snapshot of real-time flight data (IMU readings, attitude, setpoints, arming status, etc.). Note: This command respects the global logging state (`log on/off`).
*   `log on/off`: Enables or disables all logging output from the flight controller, including `debug` commands.
*   `save`: Saves the current settings to the ESP32's non-volatile memory (flash). Settings are persistent across reboots.
*   `reset`: Resets all settings to their default values and saves them to flash.
*   `reboot`: Reboots the ESP32 flight controller.
*   `calibrate_imu`: Triggers a manual calibration of the MPU6050 IMU sensor. Ensure the drone is level and still during calibration.
*   `motor.idle_speed`: Get/Set the minimum throttle percentage for motors when armed (e.g., `set motor.idle_speed 4.0`).
*   `rx.protocol`: Get/Set receiver protocol (0:IBUS, 1:PPM).
*   `imu.protocol`: Get/Set IMU protocol (0:MPU6050).
*   `madgwick.sample_freq`: Get/Set Madgwick filter sample frequency (Hz).
*   `madgwick.beta`: Get/Set Madgwick filter beta parameter.
*   `rx.map.<input>`: Get/Set receiver channel for a flight control input (e.g., `set rx.map.roll 0`). Inputs: THROTTLE, ROLL, PITCH, YAW, ARM_SWITCH, FAILSAFE_SWITCH, FLIGHT_MODE_SWITCH.
*   `help`: Displays a list of all available commands and their descriptions.
*   `exit`: Deactivates the CLI and returns to normal serial logging (if enabled).

### Example Session

```
--- ESP32_FC CLI Activated ---
========================================
    ______ _____ ____    _______  __
   / ____// ___// __ \  / ____/ |/ /
  / __/   \__ \/ / / / /_   |   / 
 / /___  ___/ / /_/ / / __/  /   |  
/_____/ /____/\____/ /_/    /_/|_|  
         Flight Controller CLI         
========================================

Available commands:
  get <parameter>       - Get a specific setting value.
  set <param> <value>   - Set a new value for a setting.
  dump                  - Print all current settings.
  debug                 - Print a snapshot of runtime flight data.
  log on/off            - Enable or disable all logging output.
  save                  - Save current settings to flash memory.
  reset                 - Reset all settings to defaults.
  reboot                - Reboot the flight controller.
  calibrate_imu         - Manually trigger IMU calibration.
  help                  - Show this help message.
  exit                  - Deactivate the CLI.

ESP32_FC > get pid.roll.kp
0.800
ESP32_FC > set pid.roll.kp 0.950
Set pid.roll.kp to 0.950
ESP32_FC > get motor.idle_speed
4.0
ESP32_FC > set motor.idle_speed 5.5
Set motor.idle_speed to 5.5
ESP32_FC > set rx.map.roll 0
Set rx.map.roll to 0
ESP32_FC > set rx.map.throttle 1
Set rx.map.throttle to 1
ESP32_FC > dump

--- [ Flight Controller Settings ] ---

--- PID Settings ---
  pid.roll.kp              : 0.950
  pid.roll.ki              : 0.001
  pid.roll.kd              : 0.050

  pid.pitch.kp             : 0.800
  pid.pitch.ki             : 0.001
  pid.pitch.kd             : 0.050

  pid.yaw.kp               : 1.500
  pid.yaw.ki               : 0.005
  pid.yaw.kd               : 0.100

  pid.integral_limit       : 400.00

--- Rate Settings ---
  rates.angle              : 30.00 (deg)
  rates.yaw                : 90.00 (deg/s)
  rates.acro               : 90.00 (deg/s)

--- Filter Settings ---
  madgwick.sample_freq     : 250.0 (Hz)
  madgwick.beta            : 0.1000

--- Receiver Settings ---
  rx.min                   : 1000
  rx.max                   : 2000
  rx.arming_threshold      : 1500
  rx.failsafe_threshold    : 1500

--- IMU Settings ---
  imu.protocol             : MPU6050

--- Receiver Channel Mapping ---
  THROTTLE                 : 1
  ROLL                     : 0
  PITCH                    : 2
  YAW                      : 3
  ARM_SWITCH               : 4
  FAILSAFE_SWITCH          : 5
  FLIGHT_MODE_SWITCH       : 6

--- Logging Settings ---
  log.interval             : 0 (ms)
  log.enabled              : true

--- Motor Settings ---
  motor.idle_speed         : 5.5 (%)

--------------------------------------
ESP32_FC > save
INFO: Settings saved. Rebooting...
(ESP32 reboots and reconnects)
--- ESP32 Flight Controller Ready ---
ESP32_FC > exit
```

---

## 🌐 Web Configurator Usage

The ESP32 Flight Controller now includes a web-based configurator for convenient tuning and interaction. This web application runs directly in your browser and communicates with the ESP32 via the Web Serial API.

### Accessing the Web Configurator

1.  **Upload Firmware:** Ensure the latest firmware (with Web Serial API support) is uploaded to your ESP32.
2.  **Open `index.html`:** Navigate to the `webui/` directory in your cloned repository and open the `index.html` file in a Web Serial API-compatible browser (e.g., Google Chrome).

### Features and Usage

*   **Connect/Disconnect:** Click the "Connect" button to establish a serial connection to your ESP32. A 2-second delay is built-in after connection to allow the ESP32 to stabilize. Click "Disconnect" to close the connection.
*   **PID Tuning:** Adjust Roll, Pitch, and Yaw Kp, Ki, and Kd values using the input fields. Click "Set PIDs & Save" to apply changes and save them to the ESP32's flash memory.
*   **CLI Command Input:** Type any CLI command (e.g., `get pid.roll.kp`, `dump`) into the input field and click "Send Command" to execute it on the ESP32.
*   **Live Data:** Toggle the "Enable Live Data" checkbox to control the streaming of real-time flight data from the ESP32. When enabled, the web app sends `log on` and periodically requests `debug` information. When disabled, it sends `log off`.
*   **Serial Output:** All communication with the ESP32 (sent commands and received responses) is displayed in the "Serial Output" area.

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
| `ImuInterface.h`          | Defines the abstract base class for all IMU sensor protocols.                   |
| `Mpu6050Imu.h/.cpp`       | Concrete implementation for the MPU6050 IMU sensor.                           |
| `MadgwickFilter.h/.cpp`   | Implements the Madgwick filter for robust attitude estimation.                  |
| `pid_controller.h/.cpp`     | PID controller logic (used by `PidProcessor`).                                  |
| `serial_logger.h/.cpp`      | Manages formatted serial output for debugging and status monitoring.            |
| `cli.h/.cpp`              | User-friendly serial command-line interface for runtime configuration.          |
| `modules/`                | Directory containing the core processing modules:                               |
| &nbsp;&nbsp;`AttitudeEstimator.h/.cpp` | Encapsulates IMU reading, calibration, and Madgwick filter for attitude estimation. |
| &nbsp;&nbsp;`SafetyManager.h/.cpp`     | Manages arming, disarming, and failsafe logic based on receiver input.          |
| &nbsp;&nbsp;`SetpointManager.h/.cpp`   | Calculates target setpoints for roll, pitch, and yaw based on receiver input and configurable channel mapping. |
| &nbsp;&nbsp;`PidProcessor.h/.cpp`      | Manages and executes the PID control loops for all axes.                        |
| &nbsp;&nbsp;`MotorMixer.h/.cpp`        | Mixes PID outputs with throttle and sends commands to the motors.               |
---

## ⚙️ Continuous Integration (CI)

The project utilizes GitHub Actions for continuous integration, ensuring code quality and compatibility. The CI workflow includes:

*   **Matrix Builds**: Compiles the firmware for various ESP32 board configurations and example sketches.
*   **Optimized Caching**: Speeds up build times by caching dependencies.
*   **Arduino Compatibility**: Validates that the project compiles correctly within the Arduino ecosystem.

---

## 📚 Libraries & Submodules

This project stands on the shoulders of giants. The core hardware interaction is handled by these libraries, included as Git submodules:

*   [**DShotRMT**](libraries/DShotRMT): A fantastic library for generating DShot signals using the ESP32's RMT peripheral.
*   [**ESP32_MPU6050**](libraries/ESP32_MPU6050): A driver for the MPU6050 IMU.
*   [**FlyskyIBUS**](libraries/FlyskyIBUS): A lightweight library for decoding the Flysky i-BUS protocol.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for full details.