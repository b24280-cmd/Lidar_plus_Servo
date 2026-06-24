# Lidar + Servo ROS 2 Workspace

A full 3D scanning pipeline built on ROS 2 Humble. A servo-tilted RPLidar sweeps through a configurable arc, the resulting rings are stitched into a 3D point cloud in real time, and a suite of post-processing nodes lets you clean, segment, and align those scans.

## Packages

| Package | Purpose |
|---|---|
| `rplidar_ros` | SLAMTEC RPLidar driver (A1–A3, S1–S3, T1, C1) |
| `servo_serial_reader` | Two-way serial bridge between Arduino and ROS 2 |
| `servo_tf_demo` | Servo sweep state machine + live TF broadcaster + system launch |
| `pointcloud_builder` | Accumulate LaserScan rings into a 3D PointCloud2; post-processing nodes |
| `phyphox_bridge` | Stream phone IMU / GPS / magnetometer / light sensor into ROS 2 |

---

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble installed and sourced
- `colcon` build tool
- Python 3.10+

---

## System Dependencies

```bash
# Serial communication (servo bridge + phyphox bridge)
sudo apt install python3-serial

# Numerics
sudo apt install python3-numpy

# Point cloud post-processing (RANSAC, ICP, MLS, SOR)
pip install open3d

# PointCleanNet neural denoiser
pip install torch scikit-learn
```

---

## Setup

```bash
cd ~/ros2_ws/src
git clone https://github.com/b24280-cmd/Lidar_plus_Servo.git .

cd ~/ros2_ws
colcon build
source install/setup.bash
```

Add the source line to `~/.bashrc` to avoid running it every session.

---

## Running the Full Scan Pipeline

The `servo_tf_demo` package ships a single launch file that starts everything in the correct order:

```bash
ros2 launch servo_tf_demo system_launch.py
```

What it starts (sequenced):

| t | Nodes |
|---|---|
| 0 s | RPLidar S3 driver, serial bridge to Arduino |
| 2 s | Servo manager (waits for serial to be ready) |
| 4 s | Cloud builder, RViz2, save_cloud |

### Launch arguments

```bash
ros2 launch servo_tf_demo system_launch.py \
  center_angle:=127 \   # PWM value where the servo points straight ahead
  scan_offset:=28 \     # degrees swept each side of centre (±28° default)
  sweep_speed:=10.0 \   # degrees per second
  step_size:=1.0        # degrees per timer tick (use < 1.0 for finer sampling)
```

The servo mounts on top of the lidar. When the sweep completes (CENTER → MAX → CENTER → MIN → CENTER), the cloud is frozen and saved automatically to `~/Downloads/scan.ply`.

---

## Package Details

### `rplidar_ros`

Official SLAMTEC driver. Publishes `/scan` (`sensor_msgs/LaserScan`).

Supported models: A1, A2 (M7/M8/M12), A3, S1, S2, S2E, S3, T1, C1.

```bash
# Run lidar only (without the full pipeline)
ros2 launch rplidar_ros rplidar_s3_launch.py
```

Serial port permissions (run once):

```bash
cd src/rplidar_ros
source scripts/create_udev_rules.sh
```

---

### `servo_serial_reader`

Bridges the Arduino servo controller to ROS 2 over USB serial.

- Subscribes to `/servo_command` (`std_msgs/String`) — forwards commands to Arduino
- Publishes `/servo_serial` (`std_msgs/String`) — relays Arduino feedback to ROS 2

Hardcoded port: `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` at 115200 baud. Change `serial_publisher.py` if your device ID differs.

```bash
ros2 run servo_serial_reader serial_publisher
```

---

### `servo_tf_demo`

Runs the sweep state machine and keeps the TF tree up to date.

**State machine:** CENTER → MAX → CENTER → MIN → CENTER, then parks and publishes `True` on `/scan_complete`.

**TF frames published:**

| Parent | Child | Type |
|---|---|---|
| `map` | `base_link` | static |
| `base_link` | `servo_link` | static (configurable offset/rotation) |
| `servo_link` | `laser` | dynamic (updates on each `ANGLE_SET:` reply from Arduino) |

```bash
ros2 run servo_tf_demo servo_manager
```

---

### `pointcloud_builder`

Seven nodes covering the full scan-to-analysis workflow:

#### `cloud_builder` — live accumulator

Subscribes to `/scan`, looks up the `map → laser` TF on each ring, rotates 2D points into 3D map coordinates, and accumulates them into `/full_cloud` (`PointCloud2`). Freezes the cloud when `/scan_complete` fires.

```bash
ros2 run pointcloud_builder cloud_builder
```

#### `save_cloud` — auto-save to PLY

Waits for `/scan_complete`, then saves the next `/full_cloud` to `~/Downloads/scan.ply` via Open3D and shuts down.

```bash
ros2 run pointcloud_builder save_cloud
```

#### `ransac_node` — plane segmentation

Loads `~/Downloads/scan.ply`, iteratively segments up to 5 dominant planes with RANSAC, colours each plane a different colour, and publishes on `/ransac/result`.

```bash
ros2 run pointcloud_builder ransac_node
```

#### `sor_node` — Statistical Outlier Removal

Removes points whose mean distance to their K nearest neighbours exceeds 1 standard deviation. Publishes kept points on `/sor/clean` (white) and removed noise on `/sor/noise` (red).

```bash
ros2 run pointcloud_builder sor_node
```

#### `mls_node` — Moving Least Squares smoothing

Projects each point onto a locally fitted tangent plane (PCA of K=20 neighbours), removing noise perpendicular to surfaces. Publishes on `/mls/smoothed`.

```bash
ros2 run pointcloud_builder mls_node
```

#### `icp_node` — multi-scan ICP alignment

Aligns two PLY scans (`scan_a` + `scan_b`) using point-to-plane ICP with 7 rotation seed candidates (±90° on each axis + identity) to handle large orientation differences. Publishes four coloured clouds on `/icp/scan_a`, `/icp/scan_b_raw`, `/icp/scan_b_aligned`, `/icp/merged`.

```bash
ros2 run pointcloud_builder icp_node \
  --ros-args -p scan_a:=/path/to/reference.ply -p scan_b:=/path/to/new.ply
```

#### `pointcleannet_node` — neural outlier removal

Uses a reimplemented PointCleanNet (PCPNet architecture) to score each point by outlier probability and removes the top 15%. Requires pretrained weights at `~/ros2_ws/pointcleannet_weights/PointCleanNetOutliers_model.pth`. Runs on GPU if available.

```bash
ros2 run pointcloud_builder pointcleannet_node
```

---

### `phyphox_bridge`

Turns any Android or iOS phone running [Phyphox](https://phyphox.org/) into a wireless sensor for ROS 2. The phone must have "Remote access" enabled in the Phyphox experiment.

**Topics published:**

| Topic | Type | Sensor |
|---|---|---|
| `imu/data` | `sensor_msgs/Imu` | Accelerometer + gyroscope + orientation |
| `imu/mag` | `sensor_msgs/MagneticField` | Magnetometer (μT → T) |
| `fix` | `sensor_msgs/NavSatFix` | GPS |
| `illuminance` | `sensor_msgs/Illuminance` | Ambient light |

```bash
ros2 run phyphox_bridge phyphox_bridge \
  --ros-args -p host:=<phone-ip>:8080 -p rate_hz:=25.0
```

The phone's axes (screen frame) are published unrotated under `frame_id=phyphox_phone`. Add a `static_transform_publisher` to align them with your robot body (ROS REP-103 FLU convention).

---

## Hardware Setup Notes

- The RPLidar typically claims `/dev/ttyUSB0`; the Arduino will appear on the next available port (usually `/dev/ttyUSB1`). Use `ls /dev/ttyUSB*` after plugging in to confirm.
- The Arduino serial ID used in `servo_serial_reader` is `usb-1a86_USB_Serial-if00-port0`. Run `ls /dev/serial/by-id/` to find yours and update `serial_publisher.py` if different.
- PointCleanNet weights are not included in this repo. Copy `PointCleanNetOutliers_model.pth` to `~/ros2_ws/pointcleannet_weights/` before running that node.
