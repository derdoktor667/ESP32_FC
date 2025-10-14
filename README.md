# 🚁 ESP32 Flight Controller

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/derdoktor667/ESP32_FC/ci.yml?branch=main&style=for-the-badge) ![GitHub](https://img.shields.io/github/license/derdoktor667/ESP32_FC?style=for-the-badge)

An advanced, high-performance flight controller firmware for quadcopters, built on the ESP32 and the Arduino framework. This project features a highly modular, object-oriented C++ architecture with a clean separation between flight logic and communication.

---

## ✨ Key Features

*   **Maintainable & Extensible API**: The CLI/API command processor has been completely refactored. It now uses a data-driven 'Settings Registry' instead of brittle `if-else` chains. This makes the code cleaner, more robust, and significantly easier to extend with new settings.
*   **Safety-First API Mode**: The API mode now includes a critical safety timeout. If a client disconnects without warning, the firmware automatically returns to flight mode, ensuring the drone does not remain in a non-responsive state.
*   **User-Friendly CLI**: The command-line interface has been significantly improved with a detailed, categorized `help` menu, making it easier than ever to configure and debug the flight controller.
*   **Robust & Reliable Configuration**: Settings management has been completely overhauled. The firmware now reliably loads settings from flash and automatically saves a default configuration on the first boot, ensuring predictable behavior.
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

Use the `help` command in the CLI to see a full, up-to-date list of commands and available settings. The output will look like this:

```
--- Flight Controller CLI Help ---

[ General Commands ]
  help                 - Display this help message.
  exit                 - Deactivate CLI and return to flight mode.
  reboot               - Reboot the ESP32 flight controller.

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
    madgwick.sample_freq
    madgwick.beta

  --- Receiver Settings ---
    rx.min, rx.max
    rx.arming_threshold, rx.failsafe_threshold
    rx.protocol (0=IBUS, 1=PPM)
    rx.map.throttle, rx.map.roll, rx.map.pitch, rx.map.yaw
    rx.map.arm_switch, rx.map.failsafe_switch, rx.map.flight_mode_switch


  --- IMU Settings ---
    imu.protocol (0=MPU6050)

  --- Motor Settings ---
    motor.idle_speed
    motor.dshot_mode (DSHOT_OFF, DSHOT150, DSHOT300, DSHOT600, DSHOT1200)


--- End of Help ---
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