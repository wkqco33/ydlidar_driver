# YDLidar X4 Pro ROS 2 Driver

High-performance, ultra-lightweight ROS 2 driver for the **YDLidar X4 Pro**. Communicates directly over UART serial without external SDK dependencies, supporting both AMD64 and ARM64 (e.g. Raspberry Pi 4/5) architectures.

---

## Features

- **SDK-Free & Dependency Minimal**: Direct POSIX serial communication and in-house protocol parser with zero vendor SDK overhead.
- **Buffered High-Throughput I/O**: Chunk-based serial stream processing with sliding ring-buffers, reducing system-call overhead by >90%.
- **Robust Data Integrity**: Full hardware XOR checksum verification to filter out corrupt packets and bus noise.
- **Test-Driven Architecture (TDD)**: Decoupled core library (`ydlidar_driver_core`) thoroughly covered by automated Google Test suites.
- **Multi-Platform Support**: Ready-to-use Docker & Docker Compose configurations for AMD64 & ARM64 architectures.
- **Standard ROS 2 Interfaces**: Publishes standard `sensor_msgs/msg/LaserScan` with dynamic start/stop scan service management and auto-reconnection.

---

## Architecture Overview

```
+-------------------+      UART Serial      +----------------------+
|  YDLidar X4 Pro   | <==================> |  X4ProSerial (POSIX)  |
+-------------------+                      +----------------------+
                                                      |
                                                      v
                                           +----------------------+
                                           |  X4ProProtocol /     |
                                           |  X4ProLidar (Core)   |
                                           +----------------------+
                                                      |
                                                      v
                                           +----------------------+
                                           |  ydlidar_node        |
                                           |  (ROS 2 Lifecycle)   |
                                           +----------------------+
                                                /           \
                                               v             v
                                     Topic: /scan     Services: /start_scan
                               (sensor_msgs/LaserScan)          /stop_scan
```

---

## System Requirements

- **LiDAR Hardware**: YDLidar X4 Pro
- **Baudrate**: 128000 (standard for X4 Pro)
- **Supported ROS 2 Distributions**: Jazzy, Iron, Humble
- **Operating System**: Ubuntu 24.04 / 22.04 / Debian Bookworm

---

## Installation & Build

### 1. Local Workspace Build

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone https://github.com/wkqco33/ydlidar_driver.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build with tests
colcon build --packages-select ydlidar_driver --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

### 2. Run Automated Unit Tests (TDD)

```bash
colcon test --packages-select ydlidar_driver
colcon test-result --verbose
```

---

## Usage

### 1. Launch with ROS 2

Run with default parameters (`/dev/ttyUSB0`):
```bash
ros2 launch ydlidar_driver x4pro_launch.py
```

Override port or TF frame:
```bash
ros2 launch ydlidar_driver x4pro_launch.py port:=/dev/ttyUSB1 frame_id:=laser_link
```

### 2. Docker & Docker Compose

#### Using Helper Script:
```bash
# Build Docker image
./run_ydlidar_docker.sh --build

# Run driver container
./run_ydlidar_docker.sh -p /dev/ttyUSB0 -f laser_frame
```

#### Using Docker Compose:
```bash
docker compose up -d
```

---

## Parameters

Configuration available via `params/x4pro.yaml` or launch arguments:

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `port` | `string` | `/dev/ttyUSB0` | Serial port path connected to LiDAR |
| `frame_id` | `string` | `laser_frame` | Header TF frame ID for `LaserScan` |
| `baudrate` | `int` | `128000` | Baudrate (128000 for X4 Pro) |
| `angle_min` | `double` | `-180.0` | Minimum scan angle (degrees) |
| `angle_max` | `double` | `180.0` | Maximum scan angle (degrees) |
| `range_min` | `double` | `0.25` | Minimum valid range (meters) |
| `range_max` | `double` | `12.0` | Maximum valid range (meters) |
| `invalid_range_is_inf` | `bool` | `false` | Set invalid/out-of-range points to `inf` (true) or `0.0` (false) |

---

## ROS 2 Interfaces

### Published Topics
- `/scan` ([`sensor_msgs/msg/LaserScan`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html))
  2D polar laser scan data sampled at 1-degree resolution.

### Services
- `/start_scan` ([`std_srvs/srv/Empty`](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html))
  Starts motor rotation and activates laser streaming.
- `/stop_scan` ([`std_srvs/srv/Empty`](http://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html))
  Stops laser streaming and halts motor rotation via DTR.

---

## Open Source Guidelines

- **AI Agent Guidelines**: See [AGENTS.md](AGENTS.md) for automated agent conventions and TDD instructions.
- **Contributing**: See [CONTRIBUTING.md](CONTRIBUTING.md) for PR workflows and style rules.
- **Security**: See [SECURITY.md](SECURITY.md) for reporting vulnerabilities.
- **License**: [Apache-2.0](LICENSE).
