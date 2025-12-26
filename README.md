# UGV Beast Control System

Control scripts and autonomous navigation for the UGV Beast robot with ROS2 Humble.

---

## Table of Contents

1. [First Setup](#1-first-setup)
2. [Algorithm Structure](#2-algorithm-structure)
3. [How to Use the Algorithm](#3-how-to-use-the-algorithm)
4. [More Information](#4-more-information)

---

## 1. First Setup

### 1.1 Flash the SD Card

1. Download the original Waveshare image: https://drive.google.com/file/d/1k6fEqZpp5l7Al2_RBfg__JTVs6DdfX9U/view
2. Download the Raspberry Pi Imager
3. Flash the SD card with the following settings:
   ```
   username: ws
   password: ws
   ```

### 1.2 Initial WiFi Configuration

1. Start the Pi (it will automatically connect to the AccessPopup WiFi)
2. From your computer:
   1. Connect to `accesspopup` WiFi with password: `1234567890`
   2. Go to `192.168.50.5:8888` and press Terminal, then run:
      ```bash
      cd AccessPopup/
      sudo chmod +x installconfig.sh
      sudo ./installconfig.sh
      5  # Select "setup a new wifi"
      ```
   3. Choose the WiFi you want to connect to and follow the instructions
   4. After entering the WiFi password, use `ip a` on a terminal from the Pi itself to see its new IP address
   5. Reconnect to your WiFi, and navigate to `new_pi_ip:8888`
   6. Hit continue, then `9` to exit the WiFi configuration script

### 1.3 Enable SSH

```bash
sudo raspi-config
# Select: Interface Options → SSH → Yes → Finish
```

### 1.4 Fix Requirements File

In the `requirements.txt` file, find the line `av==10.0.0` and modify it to: `av`

### 1.5 Install System Dependencies

```bash
sudo apt update
sudo apt install -y python3-dev pkg-config
sudo apt install -y libavformat-dev
sudo apt install -y libavcodec-dev
sudo apt install -y libavdevice-dev
sudo apt install -y libavutil-dev
sudo apt install -y libswscale-dev
sudo apt install -y libavfilter-dev
sudo apt install -y libswresample-dev
sudo apt install arandr
sudo apt upgrade
```

### 1.6 Install Older Cython Version

1. Activate the virtual environment:
   ```bash
   cd ~/ugv_rpi/ugv-env/bin
   source activate
   ```

2. You will see `(ugv-env)` added to the beginning of the hostname: `(ugv-env) ws@ws:~/ugv_rpi/ugv-env/bin`

3. Install packages:
   ```bash
   sudo /home/ws/ugv_rpi/ugv-env/bin/python3 -m pip install --upgrade pip setuptools wheel --no-user
   sudo /home/ws/ugv_rpi/ugv-env/bin/python3 -m pip install "cython<3.0.0" --no-cache-dir
   ```

### 1.7 Install Docker

```bash
cd /home/ws/ugv_ws
sudo apt update && sudo apt install docker.io -y
sudo usermod -aG docker ws
```

Refresh user permissions: In the terminal, run `groups` and make sure you see `docker` listed.

### 1.8 Run Setup Script

```bash
cd ~/ugv_rpi
sudo ./setup.sh
```

### 1.9 Configure Robot Type

Modify the `config.yaml` file:

```yaml
main_type: 3
module_type: 0
robot_name: UGV Beast
use_lidar: true
```

**Configuration Options:**
- `main_type`: Robot type (1 = RaspRover, 2 = UGV Rover, 3 = UGV Beast)
- `module_type`: Module type (0 = Nothing, 1 = RoArm-M2, 2 = Camera PT)

### 1.10 Reboot

```bash
sudo reboot
```

---

## Stage 2: Install UGV Control System

### 2.1 Clone Repository

```bash
cd /home/ws
git clone https://github.com/omercoder7/ugv_cont
```

### 2.2 Build Docker Image

```bash
cd /home/ws/ugv_cont/docker
sudo chmod +x *.sh
./build_ugv_image.sh
```

This builds an ARM64 Docker image (`ugv_beast_arm64:humble`) with:
- ROS2 Humble desktop + Navigation2
- SLAM Toolbox (async mode)
- Cartographer SLAM (alternative)
- RF2O Laser Odometry
- Robot Localization (EKF)
- LDLiDAR driver (LD19)

---

## 2. Algorithm Structure

### 2.1 Overview

The UGV uses a **Frontier-based Exploration with FSM-based Obstacle Avoidance** approach for autonomous navigation.

### 2.2 Core Components

```
mission/
├── main.py               # Launcher & prerequisites checker
├── fsm_avoider.py        # FSM-based obstacle avoidance (main loop)
├── simple_cost_nav.py    # Next-Best-View (NBV) navigation
├── frontier.py           # Frontier detection (explore_lite-style)
├── exploration.py        # Visited cell tracking
├── pro_avoidance.py      # Professional avoidance (TTC, gaps)
├── ros_interface.py      # ROS2 communication bridge
├── constants.py          # Sector definitions & calibration
├── virtual_obstacles.py  # Virtual boundary obstacles
└── avoider/
    └── lidar.py          # Sector distance computation
```

### 2.3 Finite State Machine (FSM)

The navigation FSM operates in the following states:

| State | Description |
|-------|-------------|
| `STOPPED` | Idle state |
| `FORWARD` | Moving with adaptive speed |
| `AVOIDING` | Steering around obstacles |
| `BACKING_UP` | Emergency reverse when too close |

### 2.4 Frontier Explorer

Based on explore_lite and Wavefront Frontier Detector (WFD):
- Identifies unexplored area boundaries
- Cost function: `C = distance_cost - gain * frontier_size + orientation_cost`
- Prefers larger frontiers with more information gain
- Prevents circular patterns with goal history tracking

### 2.5 LiDAR Sector System

The 360° LiDAR scan is divided into 60 sectors (6° each):

| Sector Group | Range | Description |
|--------------|-------|-------------|
| FRONT_ARC | ±30° (sectors 55-4) | Primary obstacle detection |
| LEFT | +6° to +90° | Left side clearance |
| RIGHT | -84° to -6° | Right side clearance |
| BACK | ±96° to ±90° | Rear clearance |

### 2.6 Pro Obstacle Avoidance

Implements advanced obstacle avoidance with:
- **Time-to-Collision (TTC)** calculation (Nav2 MPPI-based)
- **Gap analysis** between obstacles
- **Dynamic speed limiting** based on obstacle proximity
- Safe direction recommendations through gap evaluation

### 2.7 ROS2 Topic Flow

**Standard Mode:**
```
LiDAR (/scan) → RF2O Laser Odometry → /odom → SLAM Toolbox → /map
```

**EKF Mode (Recommended):**
```
LiDAR (/scan) → RF2O → /odom_rf2o ─┐
                                    ├→ EKF Filter → /odom → SLAM → /map
IMU (/imu/data) ───────────────────┘
```

### 2.8 EKF Sensor Fusion

Extended Kalman Filter combines multiple sensors for improved localization:

| Source | Data Used |
|--------|-----------|
| RF2O Laser Odometry (`/odom_rf2o`) | Position (x, y), Velocity (vx, vy) |
| IMU (`/imu/data`) | Orientation (yaw - authoritative), Angular velocity |

**Key Design Decisions:**
- Single yaw source (IMU) prevents sensor fighting
- Differential position integration reduces drift
- 50Hz output matches robot control loop frequency

---

## 3. How to Use the Algorithm

### 3.1 Starting Autonomous Scan

```bash
cd ugv_cont
./start_ros.sh --ekf
./rviz.sh slam-ekf new_map
python3 -m mission.simple_cost_nav --duration 300 --debug-marker
```

### 3.2 Script Reference

| Script | Description |
|--------|-------------|
| `start_ros.sh` | Start ROS nodes (use `--ekf` for sensor fusion) |
| `rviz.sh` | Launch RViz visualization |
| `auto_scan.py` | Autonomous room scanning with obstacle avoidance |
| `ensure_slam.sh` | SLAM health monitor and auto-restart |

### 3.3 start_ros.sh Options

```bash
./start_ros.sh [--ekf]
```

| Option | Description |
|--------|-------------|
| (none) | Standard mode with RF2O laser odometry only |
| `--ekf` | EKF mode with IMU + laser odometry fusion (recommended) |

**What it does:**
- Restarts Docker container to clear zombie processes
- Clears FastDDS shared memory cache
- Starts sensor drivers (LiDAR, IMU, motors)
- Verifies TF tree, topics, and SLAM process
- Duration: ~45 seconds for full startup

### 3.4 rviz.sh Modes

```bash
./rviz.sh <mode> [map_name] [options]
```

| Mode | Description |
|------|-------------|
| `slam-opt` | Optimized SLAM with tuned parameters (recommended) |
| `slam-ekf` | SLAM with EKF sensor fusion |
| `slam-carto` | Google Cartographer (best accuracy) |
| `slam-simple` | SLAM without Nav2 |
| `lidar` | LiDAR-only visualization |
| `ekf` | EKF sensor fusion without SLAM |
| `map` | Load existing map for navigation |
| `nav` | Navigation mode |

**Examples:**
```bash
# Start SLAM and create a new map called "office"
./rviz.sh slam-opt office

# Start SLAM with EKF for better localization
./rviz.sh slam-ekf kitchen

# Load existing map for navigation
./rviz.sh map office
```

### 3.5 Navigation Parameters

**simple_cost_nav.py Options:**
```bash
python3 -m mission.simple_cost_nav [options]
```

| Option | Default | Description |
|--------|---------|-------------|
| `--speed` / `-s` | 0.06 | Linear speed (m/s), max: 0.12 |
| `--min-dist` / `-m` | 0.35 | Minimum obstacle distance (m) |
| `--duration` / `-d` | 60 | Scan duration (seconds), 0 = unlimited |
| `--debug-marker` | off | Enable RViz goal markers |

### 3.6 FSM Avoider Thresholds

| Parameter | Value | Description |
|-----------|-------|-------------|
| `danger_threshold` | 0.40m | Triggers avoidance |
| `backup_threshold` | 0.30m | Triggers backup |
| `commit_duration` | 1.5s | Direction commitment time |
| `grid_resolution` | 0.5m | Exploration grid cell size |

### 3.7 Calibration Constants

Located in `mission/constants.py`:

| Constant | Value | Description |
|----------|-------|-------------|
| `LIDAR_ROTATION_SECTORS` | 15 | LiDAR 270° = Robot FRONT |
| `LIDAR_FRONT_OFFSET` | 0.37m | LiDAR offset from front |
| `ROBOT_WIDTH` | 0.18m | Robot width |
| `SAFE_TURN_CLEARANCE` | 0.70m | Space needed to turn |
| `MIN_BACKUP_DISTANCE` | 0.35m | Backup threshold |

---

## 4. More Information

### 4.1 Repository Structure

```
ugv_cont/
├── start_ros.sh              # Main startup wrapper
├── rviz.sh                   # Main RViz wrapper
├── auto_scan.py              # Autonomous scanning entry point
├── ensure_slam.sh            # SLAM health monitor
│
├── scripts/                  # Main operational scripts
│   ├── start_ros.sh          # ROS startup implementation
│   ├── rviz.sh               # RViz launcher implementation
│   └── ensure_bringup.sh     # Bringup health check
│
├── mission/                  # Core navigation algorithms
│   ├── main.py               # Launcher
│   ├── fsm_avoider.py        # FSM obstacle avoidance
│   ├── simple_cost_nav.py    # NBV navigation
│   ├── frontier.py           # Frontier detection
│   └── ...
│
├── config/                   # Configuration files
│   ├── slam_toolbox_optimized.yaml
│   ├── ekf_lidar_imu.yaml
│   ├── view_slam_2d.rviz
│   └── cartographer_2d.lua
│
├── launch/                   # ROS2 launch files
│   └── bringup_ekf_simple.launch.py
│
├── tools/                    # Calibration and testing
│   ├── calibrate_*.py
│   ├── test_rotation.py
│   └── keyboard_control.py
│
├── docker/                   # Docker configuration
│   ├── Dockerfile
│   ├── docker-compose.yaml
│   └── build_ugv_image.sh
│
└── mapping/                  # Mapping utilities
    ├── robot/
    └── offline/
```

### 4.2 Docker Container Architecture

**Container Name:** `ugv_rpi_ros_humble`

**Key Mounts:**
- `/dev/ttyACM0`: Serial device for LiDAR/Motor controller
- `/tmp/.X11-unix`: X11 display for RViz
- `./ugv_data`: Persistent data storage

**Environment Variables:**
- `UGV_MODEL=ugv_beast`
- `LDLIDAR_MODEL=ld19`

### 4.3 Hardware Specifications

| Component | Model |
|-----------|-------|
| Robot | UGV Beast |
| LiDAR | LD19 (360° scan) |
| IMU | Built-in |
| CPU | ARM64 (Raspberry Pi) |

### 4.4 Troubleshooting

#### Duplicate RF2O Nodes

If RF2O nodes are duplicated (robot won't receive LiDAR information):

```bash
./start_ros.sh --ekf
```

This performs a full container restart to clear zombie processes.

#### SLAM Not Publishing

```bash
./ensure_slam.sh
# or start fresh:
./rviz.sh slam-opt new_map
```

#### Robot Spins Continuously

1. Check LiDAR is publishing: `ros2 topic echo /scan --once`
2. Verify obstacle distances are reasonable
3. Reduce `--min-dist` parameter

#### No Odometry

```bash
# Check odometry topic
docker exec ugv_rpi_ros_humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom --once"

# With EKF, also check input topics
docker exec ugv_rpi_ros_humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom_rf2o --once"
docker exec ugv_rpi_ros_humble bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /imu/data --once"
```

#### EKF Not Starting

```bash
# Check if EKF node is running
docker exec ugv_rpi_ros_humble bash -c "source /opt/ros/humble/setup.bash && ros2 node list | grep ekf"

# Check EKF logs
docker exec ugv_rpi_ros_humble cat /tmp/bringup.log
```

### 4.5 Configuration Files

| File | Purpose |
|------|---------|
| `config/slam_toolbox_optimized.yaml` | SLAM parameters (4cm resolution, aggressive loop closure) |
| `config/ekf_lidar_imu.yaml` | EKF sensor fusion (50Hz, IMU + laser odometry) |
| `config/view_slam_2d.rviz` | RViz visualization configuration |
| `config/cartographer_2d.lua` | Google Cartographer configuration |

### 4.6 SLAM Configuration Details

**`slam_toolbox_optimized.yaml` Key Settings:**
- **Resolution:** 4cm (balance between detail and stability)
- **Scan Processing:** Every scan, minimum 0.3s interval
- **Scan Matching:** Minimum 15cm / 11° travel between scans
- **Loop Closure:** Aggressive detection (min_chain_size: 3)
- **TF Publish Rate:** 50Hz
