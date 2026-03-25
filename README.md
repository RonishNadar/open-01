I’d make the README reflect what is actually in the repo today: a `robot_firmware` ESP-IDF project plus Linux developer tooling in `tools/`, with firmware modules for robot state, MPU6500 and VL53L0X drivers, HAL layers, PID/kinematics/odometry services, and FreeRTOS tasks for motor, IMU, and ToF. The setup flow installs ESP-IDF v5.4 and a Tk-based launcher for build/flash/monitor, and the config currently shows firmware version `0.1.0`, an ESP32-S2-centered setup, three ToF sensors, an MPU6500 IMU, and UART settings for a Raspberry Pi link. Apache-2.0 is the repo license. ([GitHub][1])

Paste this into `README.md`:

````md
# OPEN-01

**OPEN-01** is an open-source, low-cost robotics platform focused on embedded sensing, motion control, and navigation firmware.

This repository currently contains the **embedded firmware stack** and **developer tooling** for the platform:
- ESP-IDF firmware for the robot controller
- sensor, motor, and state-management modules
- Linux setup/build/flash tooling
- a lightweight Python launcher for day-to-day firmware development

---

## What this repo is

This is the **firmware side** of OPEN-01.

The codebase is structured around an ESP-IDF application (`robot_firmware`) with modular layers for:
- shared robot state
- hardware drivers
- hardware abstraction
- control and estimation services
- periodic FreeRTOS tasks

If you are interested in embedded robotics, motor control, IMU/ToF integration, odometry, or building a clean firmware architecture for a mobile robot, this repo is meant to be a practical starting point.

---

## Current capabilities

At its current stage, OPEN-01 includes:

- **Motor control pipeline**
  - motor HAL
  - motor controller service
  - PID support
  - task-based control loop

- **Sensing**
  - **MPU6500** IMU support
  - **VL53L0X** ToF sensor support
  - multi-direction ranging layout in firmware config

- **State and estimation**
  - shared robot state module
  - kinematics utilities
  - odometry utilities

- **Developer tooling**
  - one-command Linux setup script
  - ESP-IDF environment bootstrap
  - serial-port-aware GUI launcher for build / flash / monitor workflows

---

## Tech stack

- **Language:** C / C++ / Python / Shell
- **Firmware framework:** **ESP-IDF**
- **RTOS:** **FreeRTOS**
- **Target workflow:** ESP32 firmware development
- **Host tooling:** Linux + Python Tkinter launcher

---

## Repository structure

```text
open-01/
├── robot_firmware/
│   ├── CMakeLists.txt
│   └── main/
│       ├── common/
│       │   ├── robot_state.cpp
│       │   └── robot_state.h
│       ├── config/
│       │   └── config.h
│       ├── drivers/
│       │   ├── drv_mpu6500.cpp
│       │   ├── drv_mpu6500.h
│       │   ├── drv_vl53l0x.cpp
│       │   └── drv_vl53l0x.h
│       ├── hal/
│       │   ├── hal_imu.cpp
│       │   ├── hal_imu.h
│       │   ├── hal_motor.cpp
│       │   ├── hal_motor.h
│       │   ├── hal_tof.cpp
│       │   └── hal_tof.h
│       ├── services/
│       │   ├── kinematic.cpp
│       │   ├── kinematic.h
│       │   ├── motor_controller.cpp
│       │   ├── motor_controller.h
│       │   ├── odometry.cpp
│       │   ├── odometry.h
│       │   ├── pid.cpp
│       │   └── pid.h
│       ├── tasks/
│       │   ├── task_imu.cpp
│       │   ├── task_imu.h
│       │   ├── task_motor.cpp
│       │   ├── task_motor.h
│       │   ├── task_tof.cpp
│       │   └── task_tof.h
│       ├── CMakeLists.txt
│       └── main.cpp
├── tools/
│   ├── launcher.py
│   ├── setup.sh
│   └── README_ESP-IDF_Tool.md
├── LICENSE
└── README.md
````

---

## Firmware architecture

The firmware follows a layered structure to keep the code maintainable and scalable:

### `config/`

Central place for firmware constants:

* firmware version
* GPIO assignments
* I2C/UART configuration
* task periods and priorities
* robot geometry
* PID defaults

### `common/`

Shared robot data structures and state handling.

### `drivers/`

Low-level device drivers for hardware peripherals and sensors.

### `hal/`

Hardware abstraction layer that separates board-specific interactions from higher-level logic.

### `services/`

Reusable robotics logic such as:

* PID control
* kinematics
* odometry
* motor control coordination

### `tasks/`

FreeRTOS task entry points for periodic runtime behavior.

### `main.cpp`

Bootstraps firmware startup, initializes shared state, and starts the core runtime tasks.

---

## Hardware notes

The current configuration suggests a controller and sensor stack centered around:

* **MPU6500** IMU
* **VL53L0X** ToF sensors
* motor PWM + direction control
* UART link for companion-computer integration
* configurable robot geometry and encoder parameters

This keeps the platform practical for embedded robotics experiments involving:

* motion control
* local sensing
* state estimation
* higher-level autonomy integration later

---

## Getting started

## 1) Clone the repository

```bash
git clone https://github.com/RonishNadar/open-01.git
cd open-01
```

## 2) Run the one-time setup

```bash
chmod +x tools/setup.sh
./tools/setup.sh
```

This setup script is intended to:

* install system dependencies
* install **ESP-IDF v5.4**
* set up shell access for ESP-IDF
* configure USB permissions
* install Python GUI dependencies

---

## GUI workflow

Launch the firmware helper GUI:

```bash
python3 tools/launcher.py
```

Recommended flow inside the launcher:

1. Select the **ESP-IDF project folder**: `robot_firmware/`
2. Choose the correct serial port
3. Set the correct target
4. Run:

   * **Build**
   * **Flash**
   * **Monitor**
   * or **Build+Flash+Monitor**

This is the fastest workflow for day-to-day iteration.

---

## CLI workflow

After setup, activate ESP-IDF and build manually:

```bash
. ~/esp/esp-idf/export.sh
cd robot_firmware
idf.py set-target esp32s2
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Use the serial device that matches your system.

---

## Project status

OPEN-01 is currently in the **firmware foundation / bring-up stage**.

Implemented foundation:

* core firmware project structure
* sensor drivers
* motor stack
* estimation/control services
* developer tooling

Scaffolded / natural next steps:

* battery task integration
* communications layer
* companion-computer interface expansion
* higher-level autonomy / ROS 2 side integration
* hardware documentation and wiring diagrams
* test and validation notes

---

## Why this project matters

A lot of robotics projects jump too quickly to high-level software and end up hiding the embedded details that actually make robots reliable.

OPEN-01 is intended to be different:

* **modular**
* **hackable**
* **embedded-first**
* **clear enough for learning**
* **structured enough to scale**

It is meant to be useful both as:

* a real robot firmware base
* and a readable reference for students, makers, and robotics engineers

---

## Contributing

Contributions are welcome.

Good contribution areas:

* hardware abstraction cleanup
* sensor validation
* motion control improvements
* testing utilities
* documentation
* calibration tooling
* companion-computer communication support

When contributing:

1. keep modules focused
2. avoid hardcoding board constants outside `config.h`
3. preserve the layered structure
4. document any hardware assumptions clearly

---

## Roadmap

* [x] ESP-IDF firmware scaffold
* [x] Shared robot state
* [x] Motor control task
* [x] IMU task
* [x] ToF task
* [x] PID / kinematics / odometry services
* [x] Linux setup script
* [x] Python firmware launcher
* [ ] Battery monitoring task
* [ ] Comms task
* [ ] Companion-computer bridge
* [ ] Full system bring-up docs
* [ ] Hardware wiring guide
* [ ] Demo videos / results
* [ ] ROS 2 integration documentation

---

## License

This project is licensed under the **Apache-2.0 License**.

See [`LICENSE`](LICENSE) for details.

```

One improvement that would make this README even stronger is adding a robot photo, wiring diagram, or a short GIF right under the title.
::contentReference[oaicite:1]{index=1}
```

[1]: https://github.com/RonishNadar/open-01 "GitHub - RonishNadar/open-01: OPEN-01 - Omnidirectional Perception & Embedded Navigation. An open-source, low-cost ROS2 robot platform. · GitHub"
