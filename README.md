# 🚁 ESP32 Flight Controller

<p align="center">
  <img src="https://img.shields.io/github/actions/workflow/status/derdoktor667/ESP32_FC/ci.yml?branch=main&style=for-the-badge" alt="Build Status"/>
  <img src="https://img.shields.io/github/license/derdoktor667/ESP32_FC?style=for-the-badge" alt="License"/>
  <img src="https://img.shields.io/badge/Platform-ESP32-purple?style=for-the-badge" alt="Platform"/>
  <img src="https://img.shields.io/badge/Framework-Arduino-00979D?style=for-the-badge" alt="Framework"/>
  <img src="https://img.shields.io/badge/Language-C++-00599C?style=for-the-badge" alt="Language"/>
</p>

<p align="center">
  An advanced, high-performance flight controller firmware for quadcopters, built on the ESP32 and the Arduino framework. This project features a highly modular, object-oriented C++ architecture and a robust communication protocol, paired with a powerful web-based configuration tool for real-time tuning and 3D visualization.
</p>

---

## ✨ Key Features

| Icon | Feature                        | Description                                                                                                                              |
| :--: | ------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `🧠` | **Modular C++ Architecture**   | Clean separation of flight logic and communication using modern C++ principles for enhanced stability and maintainability.                 |
| `💻` | **Web-Based Configurator**     | A powerful and reliable UI for real-time tuning, configuration, and a rock-solid 3D attitude visualization.      |
| `⚡` | **Robust MSP Protocol**        | Implements the MultiWii Serial Protocol (MSP) with automatic switching between V1 and V2, ensuring reliable, high-frequency data streaming. |
| `📈` | **High-Fidelity Sensor Fusion**| Features a fine-tuned digital low-pass filter and robust calibration logic that provides exceptionally clean and stable attitude data. |
| `🔌` | **Flexible Hardware Support**  | Works with MPU6050 IMUs, DShot ESCs, and multiple RC receiver protocols (iBUS, PPM), allowing for versatile hardware configurations.      |
| `💾` | **Persistent Settings**        | All tunable parameters are saved to non-volatile storage, ensuring your configuration is preserved across reboots.                         |

---

## 🏛️ Architecture & Design

This project is built from the ground up with a focus on modularity, performance, and maintainability. The firmware is not a monolithic block; instead, it is a collection of specialized modules that work together, each with a single responsibility.

### Core Principles

*   **Separation of Concerns:** Flight-critical logic (like PID processing and attitude estimation) is completely decoupled from communication and other non-essential tasks. This ensures that a high-frequency, stable flight loop is always maintained.
*   **Hardware Abstraction:** The core logic does not depend on specific hardware. It communicates with hardware (like the IMU and RC receiver) through clean interfaces, making it easy to adapt the firmware to new sensors or components.
*   **Testability:** By breaking the system into smaller, independent modules, each component can be tested in isolation, leading to a more robust and reliable system.

### Module Breakdown

The `src/` directory contains the heart of the firmware, organized into the following key areas:

| Path         | Responsibility                                                                                                                            |
| :----------- | :---------------------------------------------------------------------------------------------------------------------------------------- |
| `main/`      | Contains the top-level orchestrator classes: `FlightController`, which runs the main flight loop, and `CommunicationManager`, which handles all serial I/O. |
| `config/`    | Defines all global settings, data structures (like `FlightState`), and the logic for saving and loading parameters from flash memory.      |
| `hardware/`  | Provides the hardware abstraction layers. Each driver (e.g., `Mpu6050Imu`, `IbusReceiver`) implements a common interface (`ImuInterface`, `ReceiverInterface`). |
| `modules/`   | Contains the core flight algorithms. Each module encapsulates a specific part of the flight process, such as `AttitudeEstimator`, `PidProcessor`, and `SafetyManager`. |
| `utils/`     | A collection of reusable, low-level components like the `ComplementaryFilter` and `PidController` that are used by the higher-level modules. |

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

This project includes a powerful web-based configurator that runs locally and connects to the flight controller directly from your browser using the **Web Serial API**.

<details>
  <summary><strong>▶️ Click to expand: Running the Web App</strong></summary>

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
      *   Your browser will show a privacy warning. Click "Advanced" and "Proceed to localhost" to accept the self-signed certificate.
      *   Click the **Connect** button, select your ESP32's serial port from the list, and begin configuring!
</details>

---

## 🤖 Communication

The firmware communicates using the **MultiWii Serial Protocol (MSP)**. It boots into a silent `FLIGHT` mode and automatically enters MSP API mode as soon as it receives a valid MSP message (`$M<...` or `$X<...`).

For human interaction, a Command Line Interface (CLI) is also available:
*   **Activation**: Send `cli` over a serial monitor.
*   **Prompt**: `ESP32_FC >`
*   Use the `help` command for a full list of commands and settings.

---

## ⚙️ Configuration & Tuning

All flight parameters can be tuned in real-time via the CLI or the Web App.

#### PID Tuning Scale

To simplify PID tuning without requiring decimal points, all PID gain values (`kp`, `ki`, `kd`) are set and displayed on an integer scale.

*   **Rule**: A displayed value of `80` corresponds to a real-world gain of `0.8`.
*   **Example**: To set a P-gain of 0.8, use the command `set pid.roll.kp 80`.
*   The firmware automatically handles the conversion to its internal high-precision format.

---

## 📚 Libraries & Submodules

This project stands on the shoulders of giants. The core hardware interaction is handled by these libraries, included as Git submodules:

*   [**DShotRMT**](https://github.com/derdoktor667/DShotRMT): Generates DShot signals using the ESP32's RMT peripheral.
*   [**ESP32_MPU6050**](https://github.com/derdoktor667/ESP32_MPU6050): Driver for the MPU6050 IMU.
*   [**FlyskyIBUS**](https://github.com/derdoktor667/FlyskyIBUS): Decodes the Flysky i-BUS protocol.
*   [**MspParser**](https://github.com/derdoktor667/MspParser): A custom library for parsing and creating MSP messages.
*   [**ArduinoJson**](https://github.com/bblanchon/ArduinoJson): Efficient JSON serialization/deserialization (used by the web app).

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for full details.