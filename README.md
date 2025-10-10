# 🚁 ESP32 Flight Controller

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/derdoktor667/ESP32_FC/ci.yml?branch=main&style=for-the-badge) ![GitHub](https://img.shields.io/github/license/derdoktor667/ESP32_FC?style=for-the-badge)

An advanced flight controller firmware for quadcopters and other RC vehicles, built on the powerful ESP32 platform and the Arduino framework.

---

## ✨ Key Features

*   **Robust JSON-based API**: A dedicated, machine-readable API mode (`api`) provides clean, JSON-only communication. The firmware starts silently and only transmits data when explicitly requested, making it ideal for web applications or other programmatic clients.
*   **Advanced Web Configurator (`/webui`)**: A modern, single-page web application that automatically connects, fetches all settings, and streams live data from the flight controller. Features include:
    *   Automatic settings population on connect.
    *   Automatic live data streaming (Attitude, Status, Receiver Channels).
    *   Real-time PID adjustment and saving.
    *   A full-featured serial monitor for direct command interaction.
*   **Modular Object-Oriented Architecture**: The entire flight control logic is encapsulated in dedicated C++ classes, promoting clear responsibilities and easy extensibility.
*   **Centralized Flight State**: A single `FlightState` struct holds all dynamic data (attitude, setpoints, armed status, etc.), making the drone's current state transparent and easy to manage.
*   **Robust Flight Stabilization**: Utilizes an `ImuInterface` abstraction (initially MPU6050) and a **Madgwick filter** for precise attitude estimation, managed by the `AttitudeEstimator` module.
*   **High-Performance Motor Control**: Employs hardware-accelerated DShot for low-latency, high-resolution communication with ESCs.
*   **Flexible Control System**: Supports interchangeable receiver protocols (i-BUS, PPM) and features **configurable channel mapping**.
*   **Critical Safety Management**: Dedicated `SafetyManager` module handles arming/disarming and high-priority Failsafe logic.
*   **Centralized Configuration**: All tunable parameters (PIDs, rates, filters, etc.) are organized in a single `FlightControllerSettings` struct.
*   **Interactive CLI**: A user-friendly serial CLI (`cli`) for manual tuning and debugging, running parallel to the API mode.

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

## 💻 Interface Modes (API & CLI)

The firmware provides two distinct interface modes accessible over the serial connection. The device boots silently and waits for a mode selection command.

### API Mode (for Web App)

This is the primary mode for programmatic interaction, used by the web configurator. It ensures clean, machine-readable JSON communication.

1.  **Activation**: Send `api`.
2.  **Response**: The device acknowledges with `{"status":"api_mode_activated"}`.
3.  **Behavior**: In API mode, all output is strictly JSON. No prompts, welcome messages, or info texts are sent. Commands are sent one per line.

### CLI Mode (for Humans)

This mode is for manual debugging and configuration via a serial monitor.

1.  **Activation**: Connect to the ESP32 with a serial monitor (115200 baud). Type `cli` and press Enter.
2.  **Response**: The CLI activates and presents the `ESP32_FC >` prompt.
3.  **Behavior**: Provides interactive, human-readable feedback.

### Available Commands

*   `get <parameter>`: Retrieves the current value of a setting.
*   `set <parameter> <value>`: Sets a new value for a setting.
*   `dump`: Prints all settings in a human-readable format (CLI mode only).
*   `dumpjson`: Prints all settings as a single JSON object (API mode primary use).
*   `debug on/off`: Starts or stops the continuous stream of live flight data as JSON objects.
*   `save`: Saves the current settings to flash memory and reboots the device.
*   `reset`: Resets all settings to their defaults and reboots.
*   `reboot`: Reboots the ESP32.
*   `calibrate_imu`: Triggers a manual IMU calibration.
*   `help`: Displays the list of available commands (CLI mode only).
*   `exit`: Deactivates the CLI mode.

---

## 🌐 Web Configurator Usage

The ESP32 Flight Controller includes a powerful, modern web-based configurator that runs entirely in your browser.

### Accessing the Web Configurator

1.  **Upload Firmware:** Ensure the latest firmware is on your ESP32.
2.  **Open `index.html`:** In your cloned repository, navigate to the `webui/` directory and open `index.html` in a Web Serial API-compatible browser (e.g., Google Chrome, Edge).

### Automated Workflow

The web app is designed for a seamless experience:

1.  **Click "Connect"**: The app establishes a connection, waits for the ESP32 to stabilize, and automatically enters API mode.
2.  **Auto-Fetch Settings**: It immediately requests all settings from the flight controller, and the PID tuning fields populate with the current values from your device.
3.  **Auto-Start Live Data**: Once settings are received, the app automatically starts streaming live data, including attitude (roll, pitch, yaw), device status (armed, failsafe), and all receiver channel values.

### Features

*   **Automatic Configuration**: No manual commands needed to get started. Just connect and view.
*   **Live PID Tuning**: Adjust Roll, Pitch, and Yaw Kp, Ki, and Kd values in real-time. Click "Set PIDs & Save" to apply changes and save them to the ESP32's flash memory.
*   **Live Data Visualization**: Monitor flight status and receiver inputs at a glance.
*   **Full Serial Monitor**: A command-line interface is still provided for sending any manual commands and viewing all raw communication with the device.

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
| `cli.h/.cpp`              | Implements the dual-mode (CLI/API) command handling logic.                      |
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
