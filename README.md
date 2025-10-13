# 🚁 ESP32 Flight Controller

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/derdoktor667/ESP32_FC/ci.yml?branch=main&style=for-the-badge) ![GitHub](https://img.shields.io/github/license/derdoktor667/ESP32_FC?style=for-the-badge)

An advanced, high-performance flight controller firmware for quadcopters, built on the ESP32 and the Arduino framework. This project features a highly modular, object-oriented C++ architecture with a clean separation between flight logic and communication.

---

## ✨ Key Features

*   **Enhanced Web Application (`webapp/index.html`):**
    *   **Modern UI/UX:** Implemented a modern, clean, and dark design for a significantly enhanced user experience.
    *   **3D Quadcopter Model:** Features a dynamically generated 3D quadcopter model using Three.js primitives for live attitude visualization.
    *   **IMU Calibration:** Includes a dedicated "Calibrate IMU" button to send a serial command and reset the 3D model's orientation.
    *   **Web Serial API Integration:** Robust handling for serial connection/disconnection, automatic fetching and dynamic rendering of ESP32 settings, and the ability to send CLI commands.
    *   **CDN Module Loading:** Three.js and its addons are loaded via CDN using `importmap` for efficient module resolution.
*   **Enhanced Code Quality & Reliability**: Significant project-wide cleanup, including removal of unused code, redundant includes, and resolution of a DShot mode persistence bug, leading to a more robust and maintainable codebase.
*   **Clean Modular Architecture**: The firmware is built on two primary components: a `FlightController` class that handles only real-time flight tasks, and a `CommunicationManager` class that manages all serial I/O, providing a clear separation of concerns.
*   **Robust Tri-Mode Serial Interface**: The `CommunicationManager` provides three distinct operating modes for maximum flexibility:
    *   **FLIGHT Mode**: The default, silent mode for normal flight operations.
    *   **API Mode**: A machine-readable JSON interface optimized for programmatic clients and external applications.
    *   **CLI Mode**: A human-readable command-line interface for manual debugging.
*   **High-Performance JSON API**: The `api` mode provides a clean, JSON-only interface. Settings can be fetched with a single `get_settings` command, and a separate, high-frequency `live_data` stream provides real-time attitude and status with minimal overhead.
*   **Stack Overflow Protection**: The firmware intelligently pauses the CPU-intensive flight control pipeline when the API mode is active, dedicating resources to stable communication and preventing crashes.
*   **High-Performance Motor Control**: Employs hardware-accelerated DShot for low-latency, high-resolution communication with ESCs.
*   **Flexible Control System**: Supports interchangeable receiver protocols (i-BUS, PPM) and features **configurable channel mapping**.
*   **Centralized & Persistent Configuration**: All tunable parameters are organized in a single `FlightControllerSettings` struct and are reliably saved to and loaded from flash memory.

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

## 💻 Interface Modes

The firmware boots into a silent `FLIGHT` mode. To interact with it, you must activate one of the two interactive modes.

### API Mode (for Programmatic Clients)

This is the primary mode for programmatic interaction. It ensures clean, machine-readable JSON communication.

1.  **Activation**: Send `api` over the serial connection.
2.  **Response**: The device acknowledges with `{"status":"api_mode_activated"}` and begins streaming `live_data`.

### CLI Mode (for Humans)

This mode is for manual debugging and configuration via a serial monitor.

1.  **Activation**: Connect to the ESP32 with a serial monitor (115200 baud). Type `cli` and press Enter.
2.  **Response**: The CLI activates and presents the `ESP32_FC >` prompt.

### Available Commands

The CLI provides the following commands, categorized for clarity:

**General Commands:**
*   `help`: Display this help message.
*   `exit`: Deactivate CLI and return to flight mode.
*   `reboot`: Reboot the ESP32 flight controller.

**Settings Management:**
*   `get <parameter>`: Retrieve a specific setting or category (e.g., `get pid`, `get rx.channels`).
*   `get_settings`: Retrieves all settings as a single JSON object (API mode only).
*   `set <parameter> <value>`: Set a new value for a specified setting.
*   `dump`: Display all current flight controller settings (CLI mode only).
*   `save`: Save current settings to non-volatile memory.
*   `reset`: Reset all settings to factory defaults and save.

**Calibration Commands:**
*   `calibrate_imu`: Initiate IMU sensor calibration.

---

## 🌐 Web Application Interface

For a more user-friendly interaction, a web application is provided in the `webapp/` directory. This application utilizes the Web Serial API to connect directly to your ESP32 from a web browser, allowing you to send CLI commands, view serial output, and dynamically configure settings.

### Key WebApp Features:
*   **Modern UI/UX:** Features a modern, clean, and dark design with a refreshed blue accent color for an intuitive and aesthetically pleasing user experience.
*   **Responsive Layout:** Utilizes CSS Grid for a flexible and responsive layout, ensuring optimal viewing across various screen sizes.
*   **Always-Visible 3D Quadcopter Model:** Displays a dynamically generated 3D quadcopter model using Three.js primitives, providing live visualization of the flight controller's attitude (roll, pitch, yaw). The model is now significantly more realistic, featuring refined body, arms, motors, propellers, and landing gear, along with a clear orientation marker.
*   **Dedicated Console Tab:** Serial output and CLI command input are now organized within a dedicated "Console" tab for improved usability.
*   **Dynamic Settings with Save All:** Automatically fetches and renders all current settings from the ESP32 upon connection. A new "Save All Settings" button allows for convenient bulk saving of configuration changes.
*   **IMU Calibration Button:** A dedicated button to initiate IMU calibration and instantly reset the 3D model's orientation.
*   **Robust Serial Handling:** Implemented with `AbortController` for reliable connection and disconnection.
*   **Streamlined Interface:** The "Logging" category has been removed for a cleaner and more focused settings interface.

### How to Use the WebApp

1.  **Start a Local HTTP Server:** The Web Serial API requires a secure context (HTTPS) or a local HTTP server. The easiest way to run the web app is by using a simple HTTP server. If you have Node.js and npm installed, you can use `http-server`:
    ```bash
    npm install -g http-server
    cd /home/derdoktor667/Github/ESP32_FC/webapp
    http-server
    ```
    Then, open your web browser and navigate to the address provided by `http-server` (e.g., `http://localhost:8080`).
2.  **Connect your ESP32** to your computer via USB.
3.  **In the web app, click "Connect Serial"**. Your browser should prompt you to select a serial port. Choose the ESP32's port.
4.  **Once connected, the app will automatically enter API mode and fetch all current settings.** You can then use the CLI command input or the dynamically generated settings form to interact with your ESP32.

---

## 🛠️ Code Structure

The firmware is organized into a clean, modular, object-oriented structure within the `src/` directory:

| Module                        | Responsibility                                                                  |
| ----------------------------- | ------------------------------------------------------------------------------- |
| `ESP32_FC.ino`                | Main entry point. Creates and runs the `FlightController` and `CommunicationManager`. |
| `main/`                       | Contains the top-level orchestrator classes.                                    |
| &nbsp;&nbsp;`flight_controller.h/.cpp`    | The main `FlightController` class, orchestrating all flight-related modules.    |
| &nbsp;&nbsp;`CommunicationManager.h/.cpp` | Manages all serial communication (CLI/API) and logging.                       |
| `config/`                     | Contains global configuration and state definitions.                            |
| &nbsp;&nbsp;`config.h`                    | Central hub for hardware definitions and the `FlightControllerSettings` struct. |
| &nbsp;&nbsp;`FlightState.h`               | Defines the central `FlightState` struct.                                       |
| &nbsp;&nbsp;`settings.h/.cpp`             | Manages loading and saving settings to persistent storage.                      |
| `hardware/`                   | Contains hardware abstraction layers.                                           |
| &nbsp;&nbsp;`imu/`                      | IMU sensor interfaces and implementations.                                      |
| &nbsp;&nbsp;`receiver/`                 | RC receiver interfaces and implementations.                                     |
| `modules/`                    | Contains the core flight processing modules.                                    |
| &nbsp;&nbsp;`AttitudeEstimator.h/.cpp` | Encapsulates IMU reading and attitude estimation.                               |
| &nbsp;&nbsp;`SafetyManager.h/.cpp`     | Manages arming, disarming, and failsafe logic.                                  |
| &nbsp;&nbsp;`SetpointManager.h/.cpp`   | Calculates target setpoints.                                                    |
| &nbsp;&nbsp;`PidProcessor.h/.cpp`      | Manages and executes the PID control loops.                                     |
| &nbsp;&nbsp;`MotorMixer.h/.cpp`        | Mixes PID outputs and sends commands to the motors.                             |
| `utils/`                      | Contains reusable utility components.                                           |
| &nbsp;&nbsp;`filter/`                   | Madgwick filter implementation.                                                 |
| &nbsp;&nbsp;`pid/`                      | PID controller logic.                                                           |

---

## 📚 Libraries & Submodules

This project stands on the shoulders of giants. The core hardware interaction is handled by these libraries, included as Git submodules:

*   [**DShotRMT**](libraries/DShotRMT): A fantastic library for generating DShot signals using the ESP32's RMT peripheral.
*   [**ESP32_MPU6050**](libraries/ESP32_MPU6050): A driver for the MPU6050 IMU.
*   [**FlyskyIBUS**](libraries/FlyskyIBUS): A lightweight library for decoding the Flysky i-BUS protocol.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for full details.
