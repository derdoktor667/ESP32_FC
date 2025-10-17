# 🚁 ESP32 Flight Controller v0.2.6

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/derdoktor667/ESP32_FC/ci.yml?branch=main&style=for-the-badge) ![GitHub](https://img.shields.io/github/license/derdoktor667/ESP32_FC?style=for-the-badge)

An advanced, high-performance flight controller firmware for quadcopters, built on the ESP32 and the Arduino framework. This project features a highly modular, object-oriented C++ architecture with a clean separation between flight logic and communication.

---

## ✨ Key Features

*   **Modular & Object-Oriented Architecture**: Clean separation of flight logic (`FlightController`) and communication (`CommunicationManager`) with encapsulated modules.
*   **Efficient JSON-based API**: Streamlined `get_settings` command and `live_data` for high-performance client interaction.
*   **Flexible Hardware Support**: MPU6050 IMU, DShot ESCs, and interchangeable RC receiver protocols (iBUS, PPM).
*   **Centralized & Persistent Settings**: All user-tunable parameters managed in `FlightControllerSettings` with robust NVS persistence.
*   **Comprehensive Refactoring**: Extensive codebase improvements for enhanced maintainability, readability, and memory safety through `std::unique_ptr` adoption.
*   **Web App Configurator**: A powerful web-based UI for real-time tuning and monitoring via Web Serial API.

---

## ⚙️ Development Environment / Compatibility

*   **Target Platform**: ESP32
*   **ESP-IDF Version**: v5.5 (specifically v5.5.1)
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

## 🌐 Web App Configurator

This project includes a powerful web-based configurator that runs locally and connects to the flight controller directly from your browser using the Web Serial API. It provides a modern, user-friendly interface for tuning and monitoring your drone.

### Features

*   **Live Settings Editor**: Modify flight controller settings in real-time.
*   **3D Attitude Visualization**: A live 3D model of a quadcopter visualizes the drone's roll, pitch, and yaw.
*   **Raw Serial Log**: View raw serial communication for in-depth debugging.

### Running the Web App

The web app requires a **secure context (HTTPS)** to use the Web Serial API. The repository includes a simple Python script to serve the application over HTTPS using a self-signed certificate.

1.  **Generate a Self-Signed Certificate**:
    ```bash
    openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -sha256 -days 365 -nodes -subj "/C=US/ST=California/L=Mountain View/O=Google/OU=Gemini/CN=localhost"
    ```

2.  **Start the HTTPS Server**:
    ```bash
    python server.py
    ```
    You should see the output: `Serving HTTPS on https://localhost:8000`

3.  **Connect from your Browser**:
    *   Navigate to `https://localhost:8000`.
    *   Accept the self-signed certificate warning.
    *   Click **Connect**, select your ESP32's serial port, and begin configuring!

---

## 💻 Interface Modes

The firmware boots into a silent `FLIGHT` mode. To interact with it, activate one of the two interactive modes:

*   **API Mode**: For programmatic clients (JSON-based).
    *   **Activation**: Send `api` over serial.
    *   **Response**: `{"status":"api_mode_activated"}` and `live_data` streaming begins.

*   **CLI Mode**: For human interaction via serial monitor.
    *   **Activation**: Send `cli` over serial.
    *   **Prompt**: `ESP32_FC >`
    *   Use `help` for a full list of commands and settings.

---

## 📖 API Reference

Detailed JSON-based API for programmatic interaction, including live data stream and command structures.

### Example Commands

*   **Get Setting**: `get pid.roll.kp`
*   **Set Setting**: `set pid.roll.kp 850`

For comprehensive API details and examples, refer to the CLI `help` command or `GEMINI.md`.

---

## 🛠️ Code Structure

The firmware is organized into a clean, modular, object-oriented structure within the `src/` directory.

| Module                        | Responsibility                                                                  |
| ----------------------------- | ------------------------------------------------------------------------------- |
| `ESP32_FC.ino`                | Main entry point. Creates and runs the `FlightController` and `CommunicationManager`. |
| `main/`                       | Contains the top-level orchestrator classes.                                    |
| &nbsp;&nbsp;`flight_controller.h/.cpp`    | The main `FlightController` class, orchestrating all flight-related modules.    |
| &nbsp;&nbsp;`CommunicationManager.h/.cpp` | Manages all serial communication (CLI/API) and logging, extensively refactored for improved clarity and maintainability. |
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

*   [**DShotRMT**](libraries/DShotRMT): Generates DShot signals using the ESP32's RMT peripheral.
*   [**ESP32_MPU6050**](libraries/ESP32_MPU6050): Driver for the MPU6050 IMU.
*   [**FlyskyIBUS**](libraries/FlyskyIBUS): Decodes the Flysky i-BUS protocol.
*   [**ArduinoJson**](libraries/ArduinoJson): Efficient JSON serialization and deserialization.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for full details.
