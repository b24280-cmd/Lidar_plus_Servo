# Lidar + Servo ROS 2 Workspace

A complete 3D scanning pipeline built on ROS 2 Jazzy. A servo-tilted RPLidar sweeps through a configurable arc; each 2D scan ring is transformed into 3D space via live TF and accumulated into a point cloud. After the sweep finishes the cloud is saved to disk, and a suite of post-processing nodes lets you denoise, smooth, segment planes, and align multiple scans.

## Packages

| Package | Purpose |
|---|---|
| `rplidar_ros` | SLAMTEC RPLidar driver — publishes `/scan` |
| `servo_serial_reader` | Two-way serial bridge between the Arduino servo controller and ROS 2 |
| `servo_tf_demo` | Sweep state machine, live TF broadcaster, and full system launch file |
| `pointcloud_builder` | Accumulate scan rings into a 3D cloud; seven post-processing nodes |
| `phyphox_bridge` | Stream phone IMU / GPS / magnetometer / light sensor into ROS 2 over Wi-Fi |

---

## System Requirements

- **OS:** Ubuntu 24.04 LTS
- **ROS:** ROS 2 Jazzy
- **Python:** 3.12

---

## Dependencies

```bash
# ROS 2 Jazzy (if not already installed)
# https://docs.ros.org/en/jazzy/Installation.html

# Serial communication — servo bridge and phyphox bridge
sudo apt install python3-serial

# Numerics
sudo apt install python3-numpy

# Point cloud I/O and geometry (RANSAC, ICP, MLS, SOR)
pip install open3d

# PointCleanNet neural denoiser
pip install torch scikit-learn
```

---

## Setup

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/b24280-cmd/Lidar_plus_Servo.git .

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Add the last two lines to `~/.bashrc` to avoid running them every session:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running the Full Scan Pipeline

The `servo_tf_demo` package provides a single launch file that starts everything in the correct order.

**Minimal command — headless (drone / Jetson, RPLidar + Arduino connected):**

```bash
ros2 launch servo_tf_demo system_launch.py output_mode:=rosbag
```

**Full explicit command with every parameter:**

```bash
ros2 launch servo_tf_demo system_launch.py \
  output_mode:=rosbag \
  bag_path:=~/ros2_bags/scan \
  center_angle:=127 \
  scan_offset:=28 \
  sweep_speed:=10.0 \
  step_size:=1.0 \
  base_x:=0.0 \
  base_y:=0.0 \
  base_z:=1.0 \
  base_roll:=0.0 \
  base_pitch:=0.0 \
  base_yaw:=0.0 \
  servo_x:=0.0 \
  servo_y:=0.0 \
  servo_z:=-0.05 \
  servo_roll:=180.0 \
  servo_pitch:=0.0 \
  servo_yaw:=0.0
```

Startup sequence:

| Time | What starts |
|---|---|
| t = 0 s | RPLidar S3 driver + Arduino serial bridge |
| t = 2 s | Servo manager (waits for serial port to be ready) |
| t = 4 s | Cloud builder + save_cloud + RViz2 or rosbag recorder |

If any node exits unexpectedly, a crash message with a hint is printed to the terminal.

### Launch Arguments

All arguments have sensible defaults — only override what you need.

#### Sweep parameters

| Argument | Default | Description |
|---|---|---|
| `center_angle` | `127` | PWM value at which the servo points straight ahead (tune per your hardware) |
| `scan_offset` | `28` | Degrees swept each side of centre — total arc is `2 × scan_offset` |
| `sweep_speed` | `10.0` | Servo movement speed in degrees per second |
| `step_size` | `1.0` | Degrees moved per timer tick; use `< 1.0` for finer angular sampling |

#### Output mode

| Argument | Default | Description |
|---|---|---|
| `output_mode` | `rviz` | `rviz` — open RViz2 for live visualisation; `rosbag` — record topics to disk (headless, good for drone use) |
| `bag_path` | `~/ros2_bags/scan` | Output path for the rosbag (only used when `output_mode:=rosbag`). `ros2 bag` appends a timestamp suffix automatically |

Topics recorded in rosbag mode: `/scan`, `/full_cloud`, `/scan_complete`, `/tf`, `/tf_static`, `/servo_serial`.

#### Base link parameters

These place the robot/drone body (`base_link`) in the world (`map`) frame.

| Argument | Default | Description |
|---|---|---|
| `base_x` | `0.0` | X position of `base_link` relative to `map` (metres) |
| `base_y` | `0.0` | Y position of `base_link` relative to `map` (metres) |
| `base_z` | `1.0` | Z position (height) of `base_link` relative to `map` (metres) |
| `base_roll` | `0.0` | Roll of `base_link` relative to `map` (degrees) |
| `base_pitch` | `0.0` | Pitch of `base_link` relative to `map` (degrees) |
| `base_yaw` | `0.0` | Yaw of `base_link` relative to `map` (degrees) |

#### Servo mount parameters

These describe where and how the lidar+servo assembly is physically mounted relative to `base_link`.

| Argument | Default | Description |
|---|---|---|
| `servo_x` | `0.0` | X offset of `servo_link` relative to `base_link` (metres) |
| `servo_y` | `0.0` | Y offset of `servo_link` relative to `base_link` (metres) |
| `servo_z` | `-0.05` | Z offset of `servo_link` relative to `base_link` (metres) — negative because the lidar hangs below the body |
| `servo_roll` | `180.0` | Roll of `servo_link` relative to `base_link` (degrees) — 180° because the lidar is mounted upside-down |
| `servo_pitch` | `0.0` | Pitch of `servo_link` relative to `base_link` (degrees) |
| `servo_yaw` | `0.0` | Yaw of `servo_link` relative to `base_link` (degrees) |

Example — live visualisation (default), drone at 5 m altitude with 90° yaw:

```bash
ros2 launch servo_tf_demo system_launch.py \
  output_mode:=rviz \
  center_angle:=127 \
  scan_offset:=28 \
  sweep_speed:=10.0 \
  step_size:=1.0 \
  base_x:=0.0 \
  base_y:=0.0 \
  base_z:=5.0 \
  base_roll:=0.0 \
  base_pitch:=0.0 \
  base_yaw:=90.0
```

Example — headless rosbag recording (drone / Jetson use):

```bash
ros2 launch servo_tf_demo system_launch.py \
  output_mode:=rosbag \
  bag_path:=/home/user/ros2_bags/scan \
  center_angle:=127 \
  scan_offset:=28 \
  sweep_speed:=10.0 \
  step_size:=1.0 \
  base_x:=0.0 \
  base_y:=0.0 \
  base_z:=5.0 \
  base_roll:=0.0 \
  base_pitch:=0.0 \
  base_yaw:=0.0
```

---

## Package Details

### `rplidar_ros`

Official SLAMTEC driver. Publishes `/scan` (`sensor_msgs/LaserScan`).

Supported models: A1, A2 (M7 / M8 / M12), A3, S1, S2, S2E, S3, T1, C1.

Run the lidar on its own (useful for testing):

```bash
ros2 launch rplidar_ros rplidar_s3_launch.py
```

Set up udev rules so you don't need `sudo chmod` each time:

```bash
cd ~/ros2_ws/src/rplidar_ros
source scripts/create_udev_rules.sh
```

---

### `servo_serial_reader`

Bridges the Arduino servo controller to ROS 2 over USB serial at 115200 baud.

| Direction | Topic | Type | Description |
|---|---|---|---|
| ROS 2 → Arduino | `/servo_command` | `std_msgs/String` | Send `ANGLE:<n>` commands to the Arduino |
| Arduino → ROS 2 | `/servo_serial` | `std_msgs/String` | Relay `ANGLE_SET:<n>` confirmations back to ROS 2 |

The port is hardcoded to `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`. Run `ls /dev/serial/by-id/` after plugging in to find your device ID and update `serial_publisher.py` if it differs.

```bash
ros2 run servo_serial_reader serial_publisher
```

---

### `servo_tf_demo`

Runs the sweep state machine, keeps the TF tree live, and wires up the full system via the launch file.

**Sweep state machine:** CENTER → MAX → CENTER → MIN → CENTER, then parks at centre, cancels its timer, and publishes `True` on `/scan_complete`.

**TF tree:**

```
map (static)
 └── base_link (static, 1 m above map)
      └── servo_link (static, configurable mount offset/rotation)
           └── laser (dynamic — updated on every ANGLE_SET: reply from Arduino)
```

The `laser → servo_link` rotation is a pure roll equal to `actual_angle − center_angle` in radians, so every 2D scan ring is tilted correctly in 3D space.

```bash
ros2 run servo_tf_demo servo_manager
```

---

### `pointcloud_builder`

Seven nodes covering the full scan-to-analysis workflow.

---

#### `cloud_builder` — live scan accumulator

Subscribes to `/scan` (`LaserScan`). On each message, looks up the current `map → laser` TF, rotates every valid range reading into 3D map coordinates using the quaternion rotation matrix, and appends it to the growing cloud. Publishes `/full_cloud` (`PointCloud2`) every 0.5 s. Stops accumulating and freezes the cloud when `/scan_complete` fires.

```bash
ros2 run pointcloud_builder cloud_builder
```

---

#### `save_cloud` — auto-save to PLY

Waits silently until `/scan_complete` is received, then grabs the next `/full_cloud` message, converts it to an Open3D point cloud, and saves it to:

```
~/Downloads/scan.ply
```

Shuts itself down automatically after saving.

```bash
ros2 run pointcloud_builder save_cloud
```

---

#### `ransac_node` — plane segmentation

Loads `~/Downloads/scan.ply` and iteratively extracts up to 5 dominant planes using RANSAC (distance threshold 5 cm, 1000 iterations). Each plane is coloured a different colour (red → green → blue → yellow → magenta); remaining non-planar points are shown in grey. Publishes on `/ransac/result` at 1 Hz.

```bash
ros2 run pointcloud_builder ransac_node
```

Tunable constants at the top of `ransac_node.py`:

| Constant | Default | Effect |
|---|---|---|
| `DISTANCE_THRESHOLD` | `0.05` m | How close a point must be to the plane to count as an inlier |
| `MIN_POINTS` | `100` | Planes with fewer inliers are ignored |
| `MAX_PLANES` | `5` | Maximum number of planes to extract |
| `NUM_ITERATIONS` | `1000` | RANSAC iterations per plane |

---

#### `sor_node` — Statistical Outlier Removal

For each point, computes the mean distance to its K nearest neighbours. Points whose mean distance exceeds `mean + STD_RATIO × std_dev` across the whole cloud are flagged as outliers. Publishes kept points on `/sor/clean` (white) and removed noise on `/sor/noise` (red) at 1 Hz.

```bash
ros2 run pointcloud_builder sor_node
```

Tunable constants:

| Constant | Default | Effect |
|---|---|---|
| `NB_NEIGHBORS` | `20` | Neighbourhood size for distance statistics |
| `STD_RATIO` | `1.0` | Raise to keep more points; lower to remove more aggressively |

---

#### `mls_node` — Moving Least Squares smoothing

For each point, fits a local tangent plane through its K=20 nearest neighbours using PCA (the plane normal is the eigenvector with the smallest eigenvalue). The point is then projected onto that plane, eliminating noise perpendicular to the surface. The entire computation is vectorised with NumPy. Publishes the smoothed cloud on `/mls/smoothed` (white) at 1 Hz.

```bash
ros2 run pointcloud_builder mls_node
```

Tunable constant: `K_NEIGHBORS` (default `20`) — higher values give smoother results at the cost of over-smoothing fine detail.

---

#### `icp_node` — multi-scan ICP alignment

Aligns two PLY scans onto a common coordinate frame. To handle large rotational offsets (including ~90° misalignments), it tests 7 rotation seeds (±90° around each axis + identity) with a fast coarse ICP pass, picks the best, then refines with multi-scale ICP at voxel sizes 4× → 2× → 1× `voxel_fine`.

Publishes four coloured clouds at 1 Hz:

| Topic | Colour | Content |
|---|---|---|
| `/icp/scan_a` | Blue | Reference scan (unchanged) |
| `/icp/scan_b_raw` | Red | Second scan before alignment |
| `/icp/scan_b_aligned` | Green | Second scan after ICP |
| `/icp/merged` | Blue + Green | Both scans overlaid |

Default scan paths: `~/Desktop/Scans_s3_rplidar/scan_lab_bp.ply` and `scan_tilted.ply`.

```bash
ros2 run pointcloud_builder icp_node \
  --ros-args \
  -p scan_a:=/path/to/reference.ply \
  -p scan_b:=/path/to/new_scan.ply \
  -p voxel_fine:=0.05
```

---

#### `pointcleannet_node` — neural outlier removal

Uses a reimplemented PointCleanNet (PCPNet architecture with spatial transformer networks) to assign an outlier probability to every point. For each point, a patch of its K=500 nearest neighbours is extracted, centred, and normalised; the network processes batches of patches and returns a score in [0, 1]. The top 15% by score are removed.

Runs on GPU automatically if CUDA is available; falls back to CPU otherwise.

Requires pretrained weights at:

```
~/ros2_ws/pointcleannet_weights/PointCleanNetOutliers_model.pth
```

Publishes clean points on `/pointcleannet/clean` (white) and removed outliers on `/pointcleannet/outliers` (red) at 1 Hz.

```bash
ros2 run pointcloud_builder pointcleannet_node
```

Tunable constants:

| Constant | Default | Effect |
|---|---|---|
| `PATCH_SIZE` | `500` | Neighbourhood size — must match the trained model |
| `BATCH_SIZE` | `256` | Larger batches are faster on GPU |
| `OUTLIER_FRACTION` | `0.15` | Top fraction removed (15% by default) |
| `MAX_POINTS` | `10000` | Cloud is subsampled if larger (GPU memory limit) |

---

### `phyphox_bridge`

Turns any Android or iOS phone running [Phyphox](https://phyphox.org/) into a wireless IMU / GPS sensor for ROS 2. Enable "Remote access" in the Phyphox experiment before launching.

**Topics published:**

| Topic | Type | Sensor | Notes |
|---|---|---|---|
| `imu/data` | `sensor_msgs/Imu` | Accelerometer + gyroscope + orientation | Orientation marked unknown if attitude unavailable |
| `imu/mag` | `sensor_msgs/MagneticField` | Magnetometer | Converted from μT to T |
| `fix` | `sensor_msgs/NavSatFix` | GPS | Horizontal/vertical accuracy mapped to covariance |
| `illuminance` | `sensor_msgs/Illuminance` | Ambient light sensor | Raw lux value |

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `host` | `10.154.225.107:8080` | Phone IP and Phyphox remote port |
| `rate_hz` | `25.0` | Polling rate in Hz |
| `timeout_s` | `1.0` | HTTP request timeout |
| `frame_id` | `phyphox_phone` | TF frame for all published messages |
| `auto_start` | `true` | Send `/control?cmd=start` to the phone on launch |
| `skip_backlog` | `true` | Ignore data recorded before the node started |
| `sync_tol_s` | `0.02` | Accel ↔ gyro pairing tolerance in seconds |

```bash
ros2 run phyphox_bridge phyphox_bridge \
  --ros-args -p host:=<phone-ip>:8080 -p rate_hz:=25.0
```

The phone's axes (screen frame: x=right, y=up, z=out-of-screen) are published unrotated under `frame_id=phyphox_phone`. Add a `static_transform_publisher` to align them with your robot body frame (ROS REP-103 FLU convention).

---

## Hardware Notes

- **RPLidar port:** typically `/dev/ttyUSB0`. Run `ls /dev/ttyUSB*` after plugging in to confirm.
- **Arduino port:** the `servo_serial_reader` looks for `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`. Run `ls /dev/serial/by-id/` to find your exact ID and update `serial_publisher.py` if it differs.
- **PointCleanNet weights:** not included in this repo. Place `PointCleanNetOutliers_model.pth` in `~/ros2_ws/pointcleannet_weights/` before running `pointcleannet_node`.
- **Post-processing nodes** (`ransac_node`, `sor_node`, `mls_node`, `icp_node`, `pointcleannet_node`) all read from `~/Downloads/scan.ply` by default — run `save_cloud` first to produce that file, or point them at an existing PLY.
