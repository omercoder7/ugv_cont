# UGV Control System

Control scripts and autonomous navigation for the UGV Beast robot with ROS2 Humble.

## Quick Start

### Recommended: EKF Mode (Best Accuracy)
```bash
# 1. Start ROS with EKF sensor fusion
./start_ros.sh --ekf

# 2. Launch RViz for visualization
./rviz.sh slam-ekf

# 3. In another terminal, run autonomous scanning
python3 auto_scan.py --duration 120
```

### Basic Operation (Without EKF)
```bash
# 1. Start ROS
./start_ros.sh

# 2. Launch RViz with SLAM
./rviz.sh slam-opt

# 3. Run autonomous scanning
python3 auto_scan.py --duration 120
```

## Scripts Overview

| Script | Description |
|--------|-------------|
| `start_ros.sh` | Start ROS nodes cleanly (use `--ekf` for sensor fusion) |
| `rviz.sh` | Launch RViz visualization (various modes available) |
| `auto_scan.py` | Autonomous room scanning with obstacle avoidance |
| `ensure_bringup.sh` | Ensures robot bringup is running |

## rviz.sh Modes

```bash
./rviz.sh <mode> [map_name] [options]
```

### Modes

| Mode | Description |
|------|-------------|
| `slam-opt` | SLAM with optimized parameters (recommended for scanning) |
| `slam` | Standard SLAM |
| `map` | Load existing map for navigation |
| `nav` | Navigation mode |
| `lidar` | Lidar-only visualization |

### Options

| Option | Description |
|--------|-------------|
| `--ekf` | Enable EKF sensor fusion (slam-opt mode only) |
| `map_name` | Name for new map or existing map to load |

### Examples

```bash
# Start SLAM and create a new map called "office"
./rviz.sh slam-opt office

# Start SLAM with EKF for better localization
./rviz.sh slam-opt kitchen --ekf

# Load existing map for navigation
./rviz.sh map office
```

## Autonomous Scanning (auto_scan.py)

Autonomous exploration using Vector Field Histogram (VFH) for obstacle avoidance.

### Usage

```bash
python3 auto_scan.py [options]
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--speed` | 0.15 | Linear speed (m/s) |
| `--angular-speed` | 0.35 | Angular speed (rad/s) |
| `--obstacle-distance` | 0.55 | Distance to trigger avoidance (m) |
| `--clear-distance` | 0.70 | Distance to resume forward motion (m) |

### Features

- **VFH Obstacle Avoidance**: Uses 36 histogram bins for smooth navigation
- **Adaptive Speed Control**: Automatically slows down near obstacles for tighter maneuvering
- **Stuck Detection**: Detects and recovers from stuck situations
- **Rotation Limiting**: Prevents excessive spinning (max ~1.5 revolutions)
- **Smooth Transitions**: Gradual speed changes for stable odometry

### Behavior States

1. **FORWARD**: Moving forward, scanning for obstacles
2. **AVOIDING**: Rotating to find clear path using VFH
3. **STUCK**: Recovery mode when robot can't progress

### Adaptive Speed Control

The robot automatically adjusts speed based on obstacle proximity:
- **Far (>1.0m)**: Full speed
- **Near (<0.5m)**: 40% speed
- **In between**: Linear interpolation

This allows the robot to drive confidently in open areas while carefully maneuvering in tight spaces.

## EKF Sensor Fusion

Extended Kalman Filter (EKF) combines multiple sensors for improved localization.

### Why Use EKF?

| Without EKF | With EKF |
|-------------|----------|
| Laser odometry only | IMU + Laser odometry fusion |
| Orientation can drift | IMU provides stable orientation |
| Single point of failure | Redundant sensing |

### EKF Configuration

The EKF fuses data from two sources:

**RF2O Laser Odometry (`/odom_rf2o`)**
- Position: x, y (differential mode - reduces drift)
- Velocity: vx, vy

**IMU (`/imu/data`)**
- Orientation: yaw (authoritative source)
- Angular velocity: yaw_rate

### Key Design Decisions

1. **Single Yaw Source**: Only IMU provides yaw to prevent sensor fighting
2. **Differential Position**: Integrates velocity instead of absolute position
3. **No IMU Acceleration**: Disabled to prevent drift accumulation
4. **50Hz Output**: Matches robot control loop frequency

### Enabling EKF

```bash
# Start ROS with EKF (recommended)
./start_ros.sh --ekf

# Then launch RViz
./rviz.sh slam-ekf
```

## Configuration Files

### slam_toolbox_optimized.yaml

Optimized SLAM parameters for indoor scanning:
- Aggressive loop closure for map consistency
- Lower resolution for faster processing
- Tuned for UGV Beast's lidar characteristics

### ekf.yaml

EKF sensor fusion configuration:
- Located at `/home/ws/ugv_ws/src/ugv_main/ugv_bringup/param/ekf.yaml`
- Copied to container at `/root/ugv_ws/install/ugv_bringup/share/ugv_bringup/param/`

## Troubleshooting

### Duplicate Nodes / Zombie Processes

Symptoms: Odometry drift, Eigensolver errors, nodes listed multiple times

```bash
# Full restart to clear zombies
./start_ros.sh --ekf
```

### SLAM Map Corruption

Symptoms: Map shows artifacts, robot position jumps

```bash
# Start fresh SLAM with new map
./rviz.sh slam-opt new_map_name
```

### Robot Spins Continuously

The autonomous scanner has rotation limits (~1.5 revolutions). If this happens:
1. Check lidar is publishing: `ros2 topic echo /scan --once`
2. Verify obstacle distances are reasonable
3. Reduce `--obstacle-distance` parameter

### EKF Not Starting

```bash
# Check if EKF node is running
docker exec ugv_rpi_ros_humble bash -c "source /opt/ros/humble/setup.bash && ros2 node list | grep ekf"

# Check EKF logs
docker exec ugv_rpi_ros_humble cat /tmp/bringup.log
```

### No Odometry

```bash
# Check odometry topic
docker exec ugv_rpi_ros_humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom --once"

# With EKF, also check input topics
docker exec ugv_rpi_ros_humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom_rf2o --once"
docker exec ugv_rpi_ros_humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /imu/data --once"
```

## Architecture

```
Host Machine (/home/ws/ugv_cont/)
├── start_ros.sh         # Start ROS nodes (use --ekf for sensor fusion)
├── rviz.sh              # Launch RViz visualization
├── auto_scan.py         # Autonomous navigation
├── ensure_bringup.sh    # Ensure bringup is running
└── *.yaml               # Config files

Docker Container (ugv_rpi_ros_humble)
├── /root/ugv_ws/        # ROS2 workspace
│   └── install/         # Built packages
├── /tmp/                # Runtime configs & logs
│   ├── slam_toolbox_optimized.yaml
│   ├── bringup.log
│   └── slam.log
```

### Topic Flow

```
Standard Mode:
  /scan → RF2O → /odom → SLAM → /map

EKF Mode:
  /scan → RF2O → /odom_rf2o ─┐
                             ├→ EKF → /odom → SLAM → /map
  /imu/data ─────────────────┘
```

## Environment Variables

Required for bringup:
- `UGV_MODEL=ugv_beast`
- `LDLIDAR_MODEL=ld19`

These are set automatically by the scripts.

## Hardware

- **Robot**: UGV Beast
- **Lidar**: LD19 (360° laser scanner)
- **IMU**: Built-in IMU (when using EKF)
- **Container**: ugv_rpi_ros_humble (ROS2 Humble)
