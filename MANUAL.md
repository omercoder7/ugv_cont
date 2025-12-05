# UGV Container Control Manual

This manual describes how to use the scripts in `/home/ws/ugv_cont/` to control the UGV robot.

---

## Quick Start

```bash
cd /home/ws/ugv_cont

# 1. Start the container
./start_ugv_service.sh

# 2. Control the robot with keyboard
./keyboard_control.sh

# 3. View sensors in RViz
./rviz.sh lidar
```

---

## Scripts Overview

| Script | Description |
|--------|-------------|
| `start_ugv_service.sh` | Start the Docker container |
| `enter_container.sh` | Enter the container shell |
| `keyboard_control.sh` | Control robot movement with keyboard |
| `stop_robot.sh` | Emergency stop the robot |
| `rviz.sh` | Launch RViz with different visualization modes |
| `auto_scan.py` | Autonomous scanning with obstacle avoidance |
| `build_ugv_image.sh` | Build the Docker image |

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

## RViz Visualization (`rviz.sh`)

Launch RViz with pre-configured views for different sensors and modes.

### Usage

```bash
./rviz.sh [mode]
```

### Available Modes

| Mode | Description |
|------|-------------|
| `lidar` | View LiDAR scan data only |
| `bringup` | Basic robot view with lidar, odometry, and TF (default) |
| `slam` | Start SLAM mapping and view 2D map |
| `slam3d` | Start SLAM mapping and view 3D map |
| `nav` | Navigation view (2D) with costmaps |
| `nav3d` | Navigation view (3D) |
| `camera` | Camera image view |
| `imu` | IMU data visualization |
| `robot` | Robot model (URDF) only |
| `full` | Full sensor view |
| `custom` | Use your own .rviz config file |
| `help` | Show help message |

### Examples

```bash
# View LiDAR data
./rviz.sh lidar

# Start SLAM and visualize mapping
./rviz.sh slam

# Basic sensor view (default)
./rviz.sh bringup
./rviz.sh           # same as above

# Navigation view
./rviz.sh nav

# Use custom RViz config
./rviz.sh custom /path/to/my_config.rviz

# Show help
./rviz.sh help
```

### Requirements

- Container must be running (`./start_ugv_service.sh`)
- X11 server running on your PC (MobaXterm has this built-in)
- Sensors must be started (`bringup_lidar.launch.py` should be running)

### X11 Display Setup

The script automatically detects your SSH client IP and sets DISPLAY. If auto-detection fails, set it manually:

```bash
# Find your PC's IP (the one SSH connected from)
echo $SSH_CONNECTION

# Set DISPLAY manually if needed
export DISPLAY=192.168.0.81:0.0  # Replace with your IP
./rviz.sh lidar
```

### Starting Sensors

Before using RViz, make sure sensors are running:

```bash
# Enter the container
./enter_container.sh

# Inside container, start sensors:
export UGV_MODEL=ugv_beast
export LDLIDAR_MODEL=ld19
ros2 launch ugv_bringup bringup_lidar.launch.py
```

Or run from host:
```bash
docker exec -d ugv_rpi_ros_humble bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
export UGV_MODEL=ugv_beast && \
export LDLIDAR_MODEL=ld19 && \
ros2 launch ugv_bringup bringup_lidar.launch.py
"
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

## ROS2 Topics Reference

| Topic | Description |
|-------|-------------|
| `/cmd_vel` | Velocity commands (Twist) |
| `/scan` | LiDAR scan data |
| `/imu/data` | IMU orientation and angular velocity |
| `/odom` | Odometry (position and velocity) |
| `/tf` | Transform tree |
| `/image_raw` | Camera image (if running) |

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

## Camera Setup

The camera requires killing the main Python app first:

```bash
# Find and kill the main app
ps aux | grep app.py
sudo kill -9 <PID>

# Start camera
docker exec -d ugv_rpi_ros_humble bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
export UGV_MODEL=ugv_beast && \
ros2 launch ugv_vision camera.launch.py
"
```

---

## Environment Variables

These are set automatically in the container:

| Variable | Value | Description |
|----------|-------|-------------|
| `UGV_MODEL` | `ugv_beast` | Robot model type |
| `LDLIDAR_MODEL` | `ld19` | LiDAR model |
| `UGV_WS_DIR` | `/home/ws/ugv_ws` | Workspace directory |

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
Make sure sensors are running first:
```bash
./rviz.sh bringup  # Check if lidar works
./rviz.sh slam     # Then try SLAM
```

---

## EKF Sensor Fusion

The robot supports EKF (Extended Kalman Filter) sensor fusion combining LiDAR odometry and IMU data for improved localization accuracy.

### How It Works

```
/odom_rf2o (LiDAR odometry) + /imu/data (IMU) → EKF → /odom (fused odometry)
```

- **LiDAR odometry** (`/odom_rf2o`): Position (x, y) and yaw from laser scan matching
- **IMU data** (`/imu/data`): Yaw rate and orientation
- **Fused output** (`/odom`): Combined odometry with improved accuracy

### Usage

```bash
# Start EKF sensor fusion only (no SLAM)
./rviz.sh ekf

# Start SLAM with EKF fusion
./rviz.sh slam
```

### Configuration

The EKF configuration is in `ekf_lidar_imu.yaml`:
- Frequency: 30 Hz
- 2D mode enabled
- Fuses position (x, y) and yaw from LiDAR
- Fuses yaw rate from IMU

---

## Autonomous Scanning (`auto_scan.py`)

Autonomous scanning mode moves the robot around while avoiding obstacles, useful for mapping a room.

### Usage

```bash
# Basic scan for 60 seconds
./auto_scan.py

# Scan for 120 seconds
./auto_scan.py --duration 120

# Custom speed (max 0.12 m/s for safety)
./auto_scan.py --speed 0.08

# Custom obstacle distance
./auto_scan.py --distance 0.6
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--duration`, `-d` | 60 | Scan duration in seconds |
| `--speed`, `-s` | 0.10 | Linear speed (max 0.12 m/s) |
| `--distance`, `-m` | 0.5 | Minimum obstacle distance (meters) |

### Controls During Scan

| Key | Action |
|-----|--------|
| `Space` | Emergency stop |
| `s` | Emergency stop |
| `Ctrl+C` | Quit |

### Algorithm

The script uses sector-based obstacle avoidance:
- Divides LiDAR scan into 12 sectors (30° each)
- Finds the clearest direction to move
- Avoids narrow passages (checks adjacent sectors)
- Treats LiDAR blind spots as obstacles
- Caps speed at 0.12 m/s for safety

### Best Practices

1. **Start SLAM first** to build a map while scanning:
   ```bash
   ./rviz.sh slam &
   sleep 10
   ./auto_scan.py --duration 120
   ```

2. **Watch the map** in RViz while scanning to verify coverage

3. **Use emergency stop** if robot heads toward danger

### Troubleshooting

**Robot not moving:**
- Check if container is running: `docker ps | grep ugv`
- Check serial port: `ls -la /dev/ttyAMA0`

**Robot drives into obstacles:**
- Increase minimum distance: `./auto_scan.py --distance 0.7`
- Check if LiDAR is publishing: `ros2 topic hz /scan`

**Robot stuck in corners:**
- The script automatically backs up and turns when stuck
- If persistently stuck, press SPACE to stop and reposition manually
