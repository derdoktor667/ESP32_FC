# 🚁 ESP32 Flight Controller v0.2.6

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/derdoktor667/ESP32_FC/ci.yml?branch=main&style=for-the-badge) ![GitHub](https://img.shields.io/github/license/derdoktor667/ESP32_FC?style=for-the-badge)

An advanced, high-performance flight controller firmware for quadcopters, built on the ESP32 and the Arduino framework. This project features a highly modular, object-oriented C++ architecture with a clean separation between flight logic and communication.

---

## ✨ Implemented
- **Web Application Configurator**: Developed a comprehensive web-based UI in the `/webapp` directory that connects via the Web Serial API. It now features an 'Info' tab as the default, displaying general software and hardware information (firmware version, system status). Other tabs (3D View, Motor Settings, PID Settings, Receiver Settings, Raw Log) are initially disabled and become available only after data is successfully received from the ESP32. The web app is served locally via a Python HTTPS server using a self-signed certificate.
  - **Web App Connection Fix**: Resolved an issue where the 'Connect' button in the web app was unresponsive due to a JavaScript syntax error and missing `releaseLock()` calls in the disconnect logic.
  - **Improved 3D Quadcopter Representation**: Enhanced the 3D model in the web app to include a central body, arms, propellers, and a flight direction marker, providing a more realistic visual representation.
  - **Web App Communication Fix (Settings JSON)**: Corrected the JSON output format for the `get_settings` command in `CommunicationManager.cpp` to properly wrap settings in a `{"settings": {...}}` object, ensuring the web app correctly receives and processes settings and activates all tabs.
- **Permanent API Mode**: Removed the 2-second safety timeout from the `CommunicationManager`. Once activated via the `api` command, the firmware now remains in API mode until the next reboot, simplifying client interaction.
- **Firmware Versioning**: Added `FIRMWARE_VERSION` constant and corresponding CLI/API commands to retrieve the current firmware version (0.2.6).
- **Major API Refactoring (Settings Registry)**: Overhauled the `CommunicationManager` to use a data-driven 'Settings Registry' for handling `get`/`set` commands. This major refactoring replaced large, hard-to-maintain `if-else` blocks with a centralized, iterable registry, dramatically improving code clarity, reducing redundancy, and making the addition of new settings trivial.
  - **Settings Registry Refinements**:
    - Consolidated the `Setting` struct definition into `CommunicationManager.h` to ensure global accessibility.
    - Integrated the `ArduinoJson` library (`ArduinoJson.h`) for robust JSON serialization and deserialization.
    - Corrected the scope and definition of `settingsRegistry` and `numSettings` by moving them to `CommunicationManager.cpp` and explicitly prefixing them with `CommunicationManager::`.
    - Replaced `s.valuePtr` with `s.value` in `CommunicationManager.cpp` for consistency with the `Setting` struct definition.
    - Ensured correct handling of `SettingType::UINT8` across all communication functions (`_printGetResponse`, `_printSetResponse`, `_handleDumpCommand`, `_handleDumpJsonCommand`).
- **Enhanced CLI/API Error Handling**: Implemented a more robust error handling mechanism for `set` commands in both CLI and API modes. Error responses now provide specific details about invalid formats, unknown parameters, invalid values, and out-of-range inputs, significantly improving user feedback and programmatic client integration.
- **API Stability Enhancement**: The `get_settings` command handler was refactored to prevent a critical race condition. A mutex flag now temporarily pauses the high-frequency `live_data` stream while the settings are being transmitted, preventing the two streams from corrupting each other and crashing the device. This ensures a robust and stable API connection, especially when fetching the initial configuration.
- **Calibrate IMU Button in Web App**: Added a 'Calibrate IMU' button to the 3D view in the web application. This button sends a `calibrate_imu` command to the ESP32 and simultaneously resets the 3D model's orientation to zero, providing a convenient way to re-calibrate and visualize the IMU data.
- **IMU Initialization Fix**: Corrected a redundant IMU initialization that occurred during startup, streamlining the boot process.
- **Robust Settings Persistence**: The logic for loading and saving settings has been completely overhauled. The firmware now correctly loads saved settings from flash, and if none are found (e.g., on first boot), it generates and saves a default configuration. This makes the initial setup and configuration management much more reliable.
- **Enhanced CLI Help**: The `help` command in the CLI has been significantly improved. It now displays a well-structured, categorized list of all available settings, making the CLI more user-friendly and self-documenting.
- **Comprehensive Project Cleanup and Professionalization**: Extensive cleanup performed across the codebase, including:
  - **Standardized Programmer Headers**: All relevant `.h` and `.cpp` files now include a consistent programmer header with author, date, and license information.
  - **Improved Code Comments**: Enhanced clarity and consistency of inline comments throughout the project, adhering to the `//` style.
  - **Removal of Redundant Comments**: Eliminated self-explanatory or outdated comments.
  - **Consistent Logging**: Standardized informational `Serial.println` messages with an "INFO: " prefix.
  - **Magic Number Elimination**: Replaced hardcoded numerical literals with named `static constexpr` constants for improved readability and maintainability. This includes defining NVS keys as `static constexpr const char*` and introducing constants for invalid receiver channel values, and a constant for microseconds to seconds conversion.
  - **Removal of Unused Variables and Redundant Includes**: Further optimization of the codebase.
  - **Elimination of Duplicate Code**: Ensured no redundant code blocks exist.
  - **Corrected JSON String Escaping**: Fixed all JSON string literal escaping issues in `CommunicationManager.cpp` for robust API communication.
  - **Corrected `Serial.readStringUntil` Usage**: Ensured proper character literal usage.
- **DShot Mode Persistence Fix**: Resolved an issue where the DShot mode setting (`dshotMode`) was not correctly persisting across reboots. The fix involved changing the internal NVS key name from `motor.dshot_mode` to `dshot_mode_val` to avoid a subtle conflict within the Preferences library, while maintaining the original CLI command interface.
- **Major Refactoring**: The entire project has been refactored to separate flight logic from communication. The new `CommunicationManager` class now handles all serial CLI and API interactions, while `FlightController` focuses solely on flight tasks.
- **Robust JSON-based API**: A dedicated, machine-readable API mode (`api`) provides clean, JSON-only communication.
- **Tri-Mode Serial Interface**: The `CommunicationManager` manages three modes (`FLIGHT`, `CLI`, `API`).
- **Automatic JSON Live Data Stream**: In `api` mode, the firmware automatically streams lean JSON objects with live flight data.
- **Robust Web App API Parsing**: Enhanced the web application's `main.js` to include explicit documentation of expected JSON API structures and implemented robust runtime checks within `handleIncomingData` to ensure reliable parsing of incoming data, improving the web app's stability and user experience.
- **MPU6050 LPF Status**: Confirmed that the MPU6050's Digital Low-Pass Filter (DLPF) is not explicitly configured in the `Mpu6050Imu` class. The `ESP32_MPU6050` library provides internal mechanisms to configure the DLPF via register access, but lacks a direct public method for this purpose.
- **Stack Overflow Prevention**: The `FlightController` pauses the main flight pipeline when the API mode is active to ensure stable communication.
- **MPU6050 LPF Configuration**: Integrated Digital Low-Pass Filter (DLPF) bandwidth setting into `FlightControllerSettings` and `Mpu6050Imu`, allowing configurable filtering for IMU data.
- **IMU Calibration Zeroing**: Modified the IMU calibration process to set the current orientation as the "horizontal zero position" by resetting the Complementary filter after sensor calibration.
- **CommunicationManager Refactoring for Readability**: Simplified `_handleSetCommand` and `_handleSerialInput` in `CommunicationManager.cpp` by extracting parsing, validation, and mode-specific logic into dedicated helper functions, significantly reducing nesting and improving clarity.
- **Complementary Filter for Attitude Estimation**: Replaced the Madgwick filter with a Complementary filter for robust attitude estimation, combining gyroscope and accelerometer data.
- **Betaflight-like Filtering with Biquad Filters**: Implemented multi-stage filtering using `BiquadFilter` for both gyroscope and accelerometer data, allowing for more effective noise reduction and improved attitude stability. A general `filterSampleFreq` parameter was introduced for all filters.

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

## 🌐 Web App Configurator

This project includes a powerful web-based configurator that runs locally and connects to the flight controller directly from your browser using the Web Serial API. It provides a modern, user-friendly interface for tuning and monitoring your drone.

### Features

*   **Live Settings Editor**: Modify flight controller settings in real-time. All available settings are automatically populated from the device.
*   **3D Attitude Visualization**: A live 3D model of a quadcopter visualizes the drone's roll, pitch, and yaw based on the `live_data` stream.
*   **Raw Serial Log**: A dedicated tab shows the raw, unfiltered serial communication with the flight controller for in-depth debugging.
*   **Robust API Data Parsing**: The web app now includes explicit documentation of expected JSON structures and robust runtime checks to ensure reliable parsing of data received from the flight controller, improving stability and user experience.

### Running the Web App

The web app requires a **secure context (HTTPS)** to use the Web Serial API. Standard HTTP servers (like `python3 -m http.server` without SSL) are not sufficient. The repository includes a simple Python script to serve the application over HTTPS using a self-signed certificate.

1.  **Generate a Self-Signed Certificate**:
    If you don't have `key.pem` and `cert.pem` files, generate them using OpenSSL. Run this command in the root of the project directory:
    ```bash
    openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -sha256 -days 365 -nodes -subj "/C=US/ST=California/L=Mountain View/O=Google/OU=Gemini/CN=localhost"
    ```

2.  **Start the HTTPS Server**:
    Run the provided Python script from the root of the project directory:
    ```bash
    python server.py
    ```
    You should see the output: `Serving HTTPS on https://localhost:8000`

3.  **Connect from your Browser**:
    *   Open a compatible browser (like Google Chrome or Microsoft Edge).
    *   Navigate to `https://localhost:8000`.
    *   Your browser will show a warning about the self-signed certificate. You must accept the risk and proceed to the page.
    *   Click the **Connect** button, select the serial port for your ESP32, and begin configuring!

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

Use the `help` command in the CLI to see a full, up-to-date list of commands and available settings. The output will look like this:

```
--- Flight Controller CLI Help ---

[ General Commands ]
  help                 - Display this help message.
  exit                 - Deactivate CLI and return to flight mode.
  reboot               - Reboot the ESP32 flight controller.
  status               - Display system status and metrics.
  version              - Display firmware version.

[ Settings Management ]
  get <parameter>      - Get a specific setting or a whole category.
  set <param> <value>  - Set a new value for a parameter.
  dump                 - Display all current settings.
  save                 - Save all settings to flash and reboot.
  reset                - Reset all settings to defaults, save, and reboot.

[ Calibration Commands ]
  calibrate_imu        - Initiate IMU sensor calibration.

[ Available Settings ]
  You can 'get' a whole category (e.g., 'get pid') or get/set a specific parameter.

  --- PID Settings ---
    pid.roll.kp, pid.roll.ki, pid.roll.kd
    pid.pitch.kp, pid.pitch.ki, pid.pitch.kd
    pid.yaw.kp, pid.yaw.ki, pid.yaw.kd
    pid.integral_limit

  --- Rate Settings ---
    rates.angle       (Max angle in ANGLE mode)
    rates.yaw         (Max yaw rate in deg/s)
    rates.acro        (Max roll/pitch rate in deg/s for ACRO mode)

  --- Filter Settings ---
    filter.comp_tau (Complementary Filter Time Constant)
    gyro.lpf_cutoff_freq (Gyroscope Low-Pass Filter Cutoff Frequency in Hz)
    accel.lpf_cutoff_freq (Accelerometer Low-Pass Filter Cutoff Frequency in Hz)
    filter.sample_freq (Filter Sample Frequency in Hz)

  --- Receiver Settings ---
    rx.min, rx.max
    rx.arming_threshold, rx.failsafe_threshold
    rx.protocol (IBUS, PPM)
    rx.map.throttle, rx.map.roll, rx.map.pitch, rx.map.yaw
    rx.map.arm_switch, rx.map.failsafe_switch, rx.map.flight_mode_switch


  --- IMU Settings ---
    imu.protocol (MPU6050)

  --- Motor Settings ---
    motor.idle_speed
    motor.dshot_mode (DSHOT_OFF, DSHOT150, DSHOT300, DSHOT600, DSHOT1200)


--- End of Help ---
```

---

## 📖 API Reference (for Programmatic Clients)

This section details the JSON-based API for programmatic interaction with the flight controller.

### General API Interaction

*   **Activation**: Send `api` over the serial connection.
*   **Response**: `{"status":"api_mode_activated"}`
*   Once activated, the device remains in API mode until it is rebooted.

### Live Data Stream

When in API mode, the device streams real-time flight data as JSON objects at `settings.printIntervalMs` intervals.

**Example `live_data` output:**
```json
{
  "live_data": {
    "attitude": {
      "roll": 1.23,
      "pitch": -0.45,
      "yaw": 87.65
    },
    "status": {
      "armed": true,
      "failsafe": false,
      "mode": "ACRO"
    },
    "motor_output": [1000, 1020, 980, 1010]
  }
}
```

### Commands

All commands are sent as plain strings over serial. Responses are JSON objects.

#### 1. Get Setting (`get <parameter>`)

*   **Request**: `get pid.roll.kp`
*   **Success Response**:
    ```json
    {
      "get": {
        "pid.roll.kp": 800
      }
    }
    ```
*   **Error Response**:
    ```json
    {
      "error": "Unknown parameter for get"
    }
    ```

#### 2. Set Setting (`set <parameter> <value>`)

*   **Request**: `set pid.roll.kp 850`
*   **Success Response**:
    ```json
    {
      "set": {
        "pid.roll.kp": 850,
        "status": "success"
      }
    }
    ```
*   **Error Responses** (with improved error messages):
    *   **Invalid Format**:
        ```json
        {
          "set": {
            "": "",
            "status": "error",
            "message": "Invalid 'set' command format. Use: set <parameter> <value>"
          }
        }
        ```
    *   **Unknown Parameter**:
        ```json
        {
          "set": {
            "unknown.param": "123",
            "status": "error",
            "message": "Unknown parameter"
          }
        }
        ```
    *   **Invalid Value (Type Mismatch)**:
        ```json
        {
          "set": {
            "pid.roll.kp": "abc",
            "status": "error",
            "message": "Invalid value. Expected: integer"
          }
        }
        ```
    *   **Value Out of Range**:
        ```json
        {
          "set": {
            "rx.map.throttle": 99,
            "status": "error",
            "message": "Value out of range. Expected: 0-15"
          }
        }
        ```

#### 3. Get All Settings (`get_settings`)

*   **Request**: `get_settings`
*   **Success Response** (example, truncated):
    ```json
    {
      "settings": {
        "pid.roll.kp": 800,
        "pid.roll.ki": 1,
        "pid.roll.kd": 50,
        "pid.integral_limit": 400.0000,
        "rates.angle": 30.0000,
        "rx.protocol": "IBUS",
        "imu.protocol": "MPU6050",
        "motor.dshot_mode": "DSHOT600",
        "rx.map.throttle": 1,
        "rx.map.roll": 0,
        "rx.map.pitch": 2,
        "rx.map.yaw": 3
        // ... other settings
      }
    }
    ```

#### 4. Get Firmware Version (`version`)

*   **Request**: `version`
*   **Success Response**:
    ```json
    {
      "version": "0.2.5"
    }
    ```

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