# UGV Container Control Manual

This manual describes how to use the scripts in `/home/ws/ugv_cont/` to control the UGV robot.

---

## Quick Start

```bash
cd /home/ws/ugv_cont

# 1. Start ROS nodes with EKF sensor fusion
./start_ros.sh

# 2. Launch RViz visualization
./rviz.sh

# 3. Run autonomous navigation
python3 -m mission.simple_cost_nav --duration 300 --debug-marker
```

---

## Scripts Overview

| Script | Description |
|--------|-------------|
| `start_ros.sh` | Start ROS nodes with EKF sensor fusion |
| `rviz.sh` | Launch RViz visualization |
| `ensure_slam.sh` | SLAM health monitor and auto-restart |
| `start_ugv_service.sh` | Start the Docker container |
| `enter_container.sh` | Enter the container shell |
| `keyboard_control.sh` | Control robot movement with keyboard |
| `stop_robot.sh` | Emergency stop the robot |
| `build_ugv_image.sh` | Build the Docker image |

---

## Starting ROS (`start_ros.sh`)

Start all ROS nodes with EKF sensor fusion.

### Usage

```bash
./start_ros.sh
```

### What It Does

1. Restarts Docker container to clear zombie processes
2. Clears FastDDS shared memory cache
3. Starts sensor drivers with EKF (LiDAR, IMU, motors)
4. Starts SLAM Toolbox for mapping
5. Verifies TF tree, topics, and processes

Duration: ~45 seconds for full startup.

### Troubleshooting

**SLAM not starting:**
```bash
./ensure_slam.sh
```

**Duplicate RF2O nodes:**
```bash
./start_ros.sh  # Full container restart clears zombies
```

---

## RViz Visualization (`rviz.sh`)

Launch RViz with SLAM and EKF visualization.

### Usage

```bash
./rviz.sh
```

### Requirements

- Run `./start_ros.sh` first to start all nodes
- X11 server running on your PC (MobaXterm has this built-in)

### X11 Display Setup

The script automatically detects your SSH client IP and sets DISPLAY. If auto-detection fails, set it manually:

```bash
# Find your PC's IP (the one SSH connected from)
echo $SSH_CONNECTION

# Set DISPLAY manually if needed
export DISPLAY=192.168.0.81:0.0  # Replace with your IP
./rviz.sh
```

### Troubleshooting

**RViz window doesn't appear:**
- Check DISPLAY variable: `echo $DISPLAY`
- Enable X11: `xhost +local:docker`
- Make sure X11 forwarding is enabled in your SSH client

**No data in RViz:**
- Check if topics are publishing: `ros2 topic list`
- Check specific topic: `ros2 topic echo /scan --once`

---

## Autonomous Navigation (`mission.simple_cost_nav`)

NBV (Next-Best-View) navigation for autonomous exploration and mapping.

### Usage

```bash
# Basic scan for 60 seconds
python3 -m mission.simple_cost_nav

# Scan for 5 minutes with debug markers
python3 -m mission.simple_cost_nav --duration 300 --debug-marker

# Custom speed (max 0.12 m/s for safety)
python3 -m mission.simple_cost_nav --speed 0.08

# Custom obstacle distance
python3 -m mission.simple_cost_nav --min-dist 0.4
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--duration`, `-d` | 60 | Scan duration in seconds (0 = unlimited) |
| `--speed`, `-s` | 0.06 | Linear speed (max 0.12 m/s) |
| `--min-dist`, `-m` | 0.35 | Minimum obstacle distance (meters) |
| `--debug-marker` | off | Enable RViz goal markers |

### Session Outputs

Each run creates a timestamped output directory:

```
mission/outputs/
└── YYYYMMDD_HHMMSS/
    ├── logs/
    │   └── navigation.log    # Full console output
    └── maps/
        ├── map.pgm           # Occupancy grid
        ├── map.yaml          # Map metadata
        └── map.png           # Map image
```

### Algorithm

The NBV Navigator:
- Scores frontier cells based on information gain, distance, and exploration history
- Avoids recently visited areas with time-decay penalties
- Uses virtual obstacles to prevent revisiting problematic areas
- Handles stuck situations with automatic backup and recovery

### Controls During Scan

| Key | Action |
|-----|--------|
| `Ctrl+C` | Graceful stop (saves map) |

### Best Practices

1. **Start ROS first** before running navigation:
   ```bash
   ./start_ros.sh
   ./rviz.sh &
   python3 -m mission.simple_cost_nav --duration 300 --debug-marker
   ```

2. **Watch the map** in RViz while scanning to verify coverage

3. **Use debug markers** to see where the robot is trying to go

### Troubleshooting

**Robot not moving:**
- Check if ROS is running: `./start_ros.sh`
- Check serial port: `ls -la /dev/ttyAMA0`

**Robot drives into obstacles:**
- Increase minimum distance: `--min-dist 0.5`
- Check if LiDAR is publishing: `ros2 topic hz /scan`

**Robot stuck in corners:**
- The algorithm automatically backs up and adds virtual obstacles
- If persistently stuck, Ctrl+C and reposition manually

---

## Keyboard Control (`keyboard_control.sh`)

Control the robot's movement using your keyboard.

### Usage

```bash
./keyboard_control.sh
```

### Controls

```
Movement Keys:
       u    i    o
       j    k    l
       m    ,    .

   u = forward + turn left
   i = forward
   o = forward + turn right
   j = turn left (in place)
   k = STOP
   l = turn right (in place)
   m = backward + turn left
   , = backward
   . = backward + turn right
```

### Speed Controls

| Key | Action |
|-----|--------|
| `q` | Increase all speeds by 10% |
| `z` | Decrease all speeds by 10% |
| `w` | Increase linear speed by 10% |
| `x` | Decrease linear speed by 10% |
| `e` | Increase angular speed by 10% |
| `c` | Decrease angular speed by 10% |

### Other Keys

| Key | Action |
|-----|--------|
| `Space` | Emergency stop |
| `k` | Stop |
| `Ctrl+C` | Quit keyboard control |

### Requirements

- Container must be running (`./start_ugv_service.sh`)
- Robot must be connected via `/dev/ttyAMA0`

### Troubleshooting

**Robot not moving:**
- Check if `ugv_driver` is running: The script starts it automatically
- Check serial connection: `ls -la /dev/ttyAMA0`
- Try emergency stop then restart: `./stop_robot.sh && ./keyboard_control.sh`

**Robot won't stop:**
- Run `./stop_robot.sh` (sends stop directly to serial port)
- If that fails, power cycle the robot

---

## Emergency Stop (`stop_robot.sh`)

Immediately stop the robot. This script sends a stop command directly to the serial port, bypassing ROS2.

### Usage

```bash
./stop_robot.sh
```

### When to Use

- Robot is moving unexpectedly
- Keyboard control is not responding
- Any emergency situation

This script works even if:
- ROS2 nodes have crashed
- `ugv_driver` is not running
- Container is unresponsive

---

## Container Management

### Start Container

```bash
./start_ugv_service.sh
```

### Enter Container

```bash
./enter_container.sh
```

Inside the container you have:
- ROS2 Humble environment sourced
- `ugv_ws` workspace sourced
- Environment variables set (`UGV_MODEL`, `LDLIDAR_MODEL`)

### Check Container Status

```bash
docker ps | grep ugv
```

### View Container Logs

```bash
docker logs ugv_rpi_ros_humble
```

### Stop Container

```bash
docker stop ugv_rpi_ros_humble
```

### Restart Container

```bash
docker restart ugv_rpi_ros_humble
```

---

## EKF Sensor Fusion

The robot uses EKF (Extended Kalman Filter) sensor fusion combining LiDAR odometry and IMU data for improved localization accuracy.

### How It Works

```
LiDAR (/scan) → RF2O → /odom_rf2o ─┐
                                    ├→ EKF Filter → /odom → SLAM → /map
IMU (/imu/data) ───────────────────┘
```

- **LiDAR odometry** (`/odom_rf2o`): Position (x, y) from laser scan matching
- **IMU data** (`/imu/data`): Yaw orientation (authoritative) and angular velocity
- **Fused output** (`/odom`): Combined odometry with improved accuracy

### Configuration

The EKF configuration is in `config/ekf_lidar_imu.yaml`:
- Frequency: 50 Hz
- 2D mode enabled
- Single yaw source (IMU) prevents sensor fighting
- Differential position integration reduces drift

---

## ROS2 Topics Reference

| Topic | Description |
|-------|-------------|
| `/cmd_vel` | Velocity commands (Twist) |
| `/scan` | LiDAR scan data |
| `/imu/data` | IMU orientation and angular velocity |
| `/odom` | Fused odometry (EKF output) |
| `/odom_rf2o` | LiDAR odometry (RF2O) |
| `/map` | SLAM occupancy grid |
| `/tf` | Transform tree |

### Useful Commands (inside container)

```bash
# List all topics
ros2 topic list

# View LiDAR data
ros2 topic echo /scan --once

# View IMU data
ros2 topic echo /imu/data --once

# Check topic frequency
ros2 topic hz /scan

# List running nodes
ros2 node list
```

---

## File Locations

| Path | Description |
|------|-------------|
| `/home/ws/ugv_cont/` | Container management scripts (host) |
| `/root/ugv_ws/` | ROS2 workspace (inside container) |
| `/dev/ttyAMA0` | Serial port for robot communication |
| `/dev/ttyACM0` | LiDAR serial port |
| `/dev/video0` | Camera device |

---

## Common Issues

### "Container not running"
```bash
./start_ugv_service.sh
```

### "Permission denied on /dev/ttyAMA0"
The container runs with `--privileged` flag, so this shouldn't happen. If it does:
```bash
sudo chmod 666 /dev/ttyAMA0
```

### "Robot keeps moving after stop"
The `ugv_driver` may have crashed. Run:
```bash
./stop_robot.sh
```
This sends stop directly to serial.

### "SLAM not working"
```bash
./ensure_slam.sh
```
This checks SLAM health and restarts if needed.

### "Duplicate RF2O nodes"
```bash
./start_ros.sh
```
Full container restart clears zombie processes.
