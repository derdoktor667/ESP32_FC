# 🚁 ESP32 Flight Controller v0.2.6

<p align="center">
  <img src="https://img.shields.io/github/actions/workflow/status/derdoktor667/ESP32_FC/ci.yml?branch=main&style=for-the-badge" alt="Build Status"/>
  <img src="https://img.shields.io/github/license/derdoktor667/ESP32_FC?style=for-the-badge" alt="License"/>
  <img src="https://img.shields.io/badge/Platform-ESP32-purple?style=for-the-badge" alt="Platform"/>
  <img src="https://img.shields.io/badge/Framework-Arduino-00979D?style=for-the-badge" alt="Framework"/>
  <img src="https://img.shields.io/badge/Language-C++-00599C?style=for-the-badge" alt="Language"/>
</p>

<p align="center">
  An advanced, high-performance flight controller firmware for quadcopters, built on the ESP32 and the Arduino framework. This project features a highly modular, object-oriented C++ architecture with a clean separation between flight logic and communication, paired with a powerful web-based configuration tool.
</p>

---

## ✨ Key Features

| Icon | Feature                        | Description                                                                                                                              |
| :--: | ------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------- |
| `🧠` | **Modular C++ Architecture**   | Clean separation of flight logic and communication using modern C++ principles for enhanced stability and maintainability.                 |
| `💻` | **Web-Based Configurator**     | A powerful and reliable UI for real-time tuning, configuration, and 3D attitude visualization, accessible from any modern web browser.      |
| `⚡` | **High-Performance JSON API**  | A streamlined, machine-readable API for programmatic control and high-frequency `live_data` streaming.                                     |
| `🔌` | **Flexible Hardware Support**  | Works with MPU6050 IMUs, DShot ESCs, and multiple RC receiver protocols (iBUS, PPM), allowing for versatile hardware configurations.      |
| `💾` | **Persistent Settings**        | All tunable parameters are saved to non-volatile storage, ensuring your configuration is preserved across reboots.                         |
| `🛡️` | **Modern C++ Design**          | Leverages features like smart pointers (`std::unique_ptr`) for exceptional memory safety and a robust, professional-grade codebase.        |

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

## 🤖 Interface Modes

The firmware boots into a silent `FLIGHT` mode. To interact with it, activate one of the two interactive modes over the serial connection:

*   **`api` Mode**: For programmatic clients and the web configurator (JSON-based).
    *   **Activation**: Send `api`
    *   **Response**: `{"status":"api_mode_activated"}` and `live_data` streaming begins.

*   **`cli` Mode**: For human interaction via a serial monitor.
    *   **Activation**: Send `cli`
    *   **Prompt**: `ESP32_FC >`
    *   Use the `help` command for a full list of commands and settings.

---

## 🛠️ Code Structure

The firmware is organized into a clean, modular, object-oriented structure within the `src/` directory.

| Module                        | Responsibility                                                                  |
| ----------------------------- | ------------------------------------------------------------------------------- |
| `ESP32_FC.ino`                | Main entry point. Creates and runs the `FlightController` and `CommunicationManager`. |
| `main/`                       | Contains the top-level orchestrator classes.                                    |
| `config/`                     | Contains global configuration, state definitions, and persistence logic.        |
| `hardware/`                   | Contains hardware abstraction layers for the IMU and RC receiver.               |
| `modules/`                    | Contains the core flight processing modules (Attitude, PID, Safety, etc.).      |
| `utils/`                      | Contains reusable utility components like filters and PID controllers.          |

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