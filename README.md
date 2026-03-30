# OPEN-01

**OPEN-01** is an open-source, low-cost differential drive robot platform focused on embedded sensing, motion control, and autonomous navigation.

This repository contains the full stack:
- ESP-IDF firmware for the ESP32-S2 robot controller
- ROS2 serial bridge and bringup for Raspberry Pi 3B
- URDF robot description with STL meshes
- RViz2 visualization stack for the dev laptop
- robot_localization EKF for sensor fusion
- Linux setup scripts for both machines

---

## What this repo is

OPEN-01 is designed as a low-cost, replicable alternative to the TurtleBot3. It is built around proven hardware and a clean layered architecture — embedded-first, with ROS2 as the autonomy layer on top.

The platform is structured around two compute nodes:

- **ESP32-S2** — handles all real-time firmware: sensors, motors, odometry, and UART telemetry
- **Raspberry Pi 3B** — runs ROS2 Humble, bridges ESP32 telemetry to ROS2 topics, and relays lidar data
- **Laptop** — runs EKF, SLAM, Nav2, and RViz2 visualization over WiFi

---

## Current capabilities

### Firmware (ESP32-S2)
- **Motor control** — FIT0441 brushless DC motors, 25kHz PWM, encoder feedback, startup kick
- **IMU** — MPU6500 (WHO_AM_I=0x70), dedicated I2C bus, accel + gyro
- **ToF ranging** — 3x VL53L0X (left, right, back), XSHUT-based address assignment
- **Odometry** — wheel encoder integration, kinematics
- **Battery monitoring** — 3S Li-Ion voltage via ADC with voltage divider
- **UART telemetry** — custom binary protocol at 460800 baud with CRC16-CCITT

### ROS2 Stack (Raspberry Pi 3B — ROS2 Humble)
- **Serial bridge node** — parses ESP32 binary protocol, publishes all sensor topics
- **Lidar node** — LDS-02 lidar via USB2LDS adapter, publishes `/scan` *(in progress)*
- **robot_state_publisher** — broadcasts full TF tree from URDF
- **Systemd autostart** — `open01.service` brings up the full stack on boot

### ROS2 Stack (Laptop — ROS2 Jazzy)
- **RViz2 visualization** — CAD model, TF frames, odometry, lidar scan
- **robot_localization EKF** — fuses `/odom` + `/imu/data` → `/odometry/filtered` at 30Hz
- **SLAM Toolbox** — *(planned)*
- **Nav2** — *(planned)*

### Published ROS2 Topics
| Topic | Type | Source |
|-------|------|--------|
| `/odom` | `nav_msgs/Odometry` | Serial bridge |
| `/imu/data` | `sensor_msgs/Imu` | Serial bridge |
| `/battery_state` | `sensor_msgs/BatteryState` | Serial bridge |
| `/tof/left` | `sensor_msgs/Range` | Serial bridge |
| `/tof/right` | `sensor_msgs/Range` | Serial bridge |
| `/tof/back` | `sensor_msgs/Range` | Serial bridge |
| `/scan` | `sensor_msgs/LaserScan` | Lidar node |
| `/joint_states` | `sensor_msgs/JointState` | Serial bridge |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF node |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / teleop |

---

## Hardware

| Component | Part |
|-----------|------|
| Microcontroller | ESP32-S2 |
| Companion computer | Raspberry Pi 3B |
| Motors | FIT0441 brushless DC with encoders |
| IMU | MPU6500 |
| ToF sensors | 3x VL53L0X |
| Lidar | LDS-02 (via USB2LDS adapter, CP210x) |
| Battery | 3S Li-Ion (12.6V full, 9.0V cutoff) |

### Pin Map (ESP32-S2)

| Function | GPIO |
|----------|------|
| I2C Bus 0 SDA (ToF) | GPIO14 |
| I2C Bus 0 SCL (ToF) | GPIO13 |
| I2C Bus 1 SDA (IMU) | GPIO5 |
| I2C Bus 1 SCL (IMU) | GPIO4 |
| XSHUT Left (0x30) | GPIO12 |
| XSHUT Back (0x31) | GPIO11 |
| XSHUT Right (0x32) | GPIO10 |
| UART TX | GPIO19 |
| UART RX | GPIO20 |
| Motor Right PWM | GPIO15 |
| Motor Right DIR | GPIO16 |
| Motor Right FG | GPIO17 |
| Motor Left PWM | GPIO21 |
| Motor Left DIR | GPIO33 |
| Motor Left FG | GPIO34 |
| Battery ADC | GPIO2 |

---

## Repository structure

```text
open-01/
├── robot_firmware/              # ESP32-S2 firmware (ESP-IDF v5.4)
│   └── main/
│       ├── config/              # GPIO, timing, robot geometry constants
│       ├── common/              # Shared robot state
│       ├── drivers/             # MPU6500, VL53L0X, motor drivers
│       ├── hal/                 # Hardware abstraction layer
│       ├── services/            # PID, kinematics, odometry, comms
│       └── tasks/               # FreeRTOS tasks (IMU, ToF, motor, comms)
├── robot_ros2/
│   └── ros2_ws/
│       └── src/
│           ├── open01_serial_bridge/   # ESP32 <-> ROS2 bridge + lidar node
│           ├── open01_bringup/         # RPi launch files
│           ├── open01_description/     # URDF + STL meshes
│           └── open01_viz/             # Laptop RViz2 launch + EKF config
├── cad/
│   └── URDFs/                   # SolidWorks URDF export (source)
├── scripts/
│   ├── setup_rpi.sh             # One-command RPi setup
│   └── setup_laptop.sh          # One-command laptop setup
├── tools/
│   ├── launcher.py              # ESP-IDF GUI launcher
│   └── setup.sh                 # ESP-IDF toolchain setup
├── LICENSE
└── README.md
```

---

## Getting started

### RPi setup (Ubuntu 22.04 + ROS2 Humble)

```bash
git clone https://github.com/RonishNadar/open-01.git
cd open-01
sudo bash scripts/setup_rpi.sh
sudo reboot
```

### Laptop setup (Ubuntu 24.04 + ROS2 Jazzy)

```bash
git clone https://github.com/RonishNadar/open-01.git
cd open-01
sudo bash scripts/setup_laptop.sh
source ~/.bashrc
```

### Firmware setup (ESP32-S2)

```bash
chmod +x tools/setup.sh
./tools/setup.sh
python3 tools/launcher.py
```

---

## Running the robot

### On RPi (auto-starts on boot via systemd)
```bash
sudo systemctl start open01.service
sudo systemctl status open01.service
```

### Manual launch on RPi
```bash
cd ~/open-01/robot_ros2/ros2_ws
source install/setup.bash
ros2 launch open01_bringup bringup.launch.py
```

### Visualization + EKF on laptop
```bash
cd ~/open-01/robot_ros2/ros2_ws
source install/setup.bash
ros2 launch open01_viz viz.launch.py
```

### Verify topics
```bash
ros2 topic list
ros2 topic hz /odometry/filtered   # should be ~30Hz
ros2 topic hz /odom                # should be ~40Hz
ros2 topic hz /scan                # should be ~5Hz
```

### Verify TF tree
```bash
ros2 run tf2_tools view_frames
```

---

## Architecture

```
ESP32-S2 (firmware)
    |  UART 460800 baud (custom binary protocol, CRC16-CCITT)
    v
Raspberry Pi 3B (ROS2 Humble)
    |-- open01_serial_bridge  ->  /odom /imu/data /battery_state /tof/* /joint_states
    |-- lidar_node            ->  /scan
    |-- robot_state_publisher ->  /tf /tf_static
    |  WiFi (DDS, ROS_DOMAIN_ID=0)
    v
Laptop (ROS2 Jazzy)
    |-- robot_localization EKF  ->  /odometry/filtered
    |-- SLAM Toolbox            ->  /map (planned)
    |-- Nav2                    ->  autonomous navigation (planned)
    |-- RViz2                   ->  visualization
```

---

## Firmware architecture

### `config/`
Central constants: GPIO assignments, I2C/UART config, task periods, robot geometry, PID defaults.

### `common/`
Shared robot state and data structures accessed across tasks.

### `drivers/`
Low-level device drivers for MPU6500, VL53L0X, and motor control.

### `hal/`
Hardware abstraction layer separating board-specific logic from services.

### `services/`
Reusable robotics logic: PID, kinematics, odometry, motor coordination, UART comms.

### `tasks/`
FreeRTOS task entry points for periodic runtime behavior. All tasks pinned to Core 0 (ESP32-S2 is single-core).

---

## Key implementation notes

- **ESP32-S2 ADC** is 13-bit — `ADC_BITWIDTH_DEFAULT`, `BATTERY_ADC_MAX = 8191.0f`
- **FIT0441 motors** use inverted PWM (255=stop, 0=full speed), require 25kHz PWM and startup kick
- **FG pins** need 5kΩ external pull-up to 3.3V (open-collector output)
- **VL53L0X** requires full Pololu init sequence (SPAD calibration, 60+ tuning registers)
- **LDS-02 checksum** is `(~sum(bytes[0:40])) & 0xFF`, `INDEX_MAX=0xDB`
- **Serial port** on Ubuntu 22.04 RPi is `/dev/ttyS0` (not `/dev/ttyAMA0`)
- **ROS2 QoS** — RPi nodes publish `BEST_EFFORT`, set RViz2 subscriptions to match
- **EKF** — IMU orientation not fused (MPU6500 has no magnetometer), only `vyaw`, `ax`, `ay` used from IMU

---

## Project status

### Firmware
- [x] ESP-IDF firmware scaffold
- [x] Shared robot state
- [x] Motor control task + PID
- [x] IMU task (MPU6500)
- [x] ToF task (3x VL53L0X)
- [x] Odometry + kinematics
- [x] Battery monitoring
- [x] UART comms task (custom binary protocol)
- [x] Linux setup script + GUI launcher

### ROS2
- [x] Serial bridge node (ESP32 <-> ROS2)
- [~] LDS-02 lidar node *(in progress — lidar bring-up)*
- [x] URDF robot description
- [x] TF tree (all frames)
- [x] RViz2 visualization
- [x] Systemd autostart
- [x] Setup scripts (RPi + laptop)
- [x] robot_localization EKF (fusing /odom + /imu/data)
- [ ] SLAM Toolbox
- [ ] Nav2
- [ ] PID gain tuning

### Documentation
- [ ] Hardware wiring guide
- [ ] Full system bring-up docs
- [ ] Demo videos

---

## Contributing

Contributions are welcome. Good areas:

- hardware abstraction cleanup
- sensor validation and calibration
- motion control improvements
- Nav2 / SLAM configuration
- documentation and wiring diagrams
- testing utilities

When contributing:
1. keep modules focused
2. avoid hardcoding constants outside `config.h`
3. preserve the layered firmware structure
4. document any hardware assumptions clearly

---

## License

Licensed under the **Apache-2.0 License**. See [`LICENSE`](LICENSE) for details.