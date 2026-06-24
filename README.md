# Lidar + Servo ROS 2 Packages

Two ROS 2 packages to complement an existing rplidar_ros setup:

- **pointcloud_builder** — subscribes to lidar scan data and builds a point cloud
- **servo_serial_reader** — reads servo position over serial and publishes it as a ROS 2 topic

## Prerequisites

- Ubuntu with ROS 2 Humble installed and sourced
- An existing ROS 2 workspace with `rplidar_ros` already built

## Setup

### 1. Install system dependencies

```bash
sudo apt install python3-serial python3-numpy
```

### 2. Clone into your workspace

Replace `~/your_ws` with whatever your workspace is actually called.

```bash
cd ~/your_ws/src
git clone https://github.com/b24280-cmd/Lidar_plus_Servo.git
```

### 3. Build only the two new packages

```bash
cd ~/your_ws
colcon build --packages-select pointcloud_builder servo_serial_reader
```

### 4. Source the workspace

```bash
source ~/your_ws/install/setup.bash
```

Add that line to your `~/.bashrc` if you haven't already.

## Notes

- The servo serial reader expects the Arduino/servo controller on a USB serial
  port. Check which port it appears on with `ls /dev/ttyUSB*` after plugging
  in, and pass it as a parameter when launching the node.
- `/dev/ttyUSB0` is typically taken by the RPLidar — the servo controller will
  likely appear as `/dev/ttyUSB1` or `/dev/ttyUSB2`.
