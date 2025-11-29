# UGV Container Usage Guide

## Quick Start

### 1. Start the Container
```bash
cd /home/ws/ugv_cont
./start_ugv_service.sh
```

### 2. Enter the Container
```bash
./enter_container.sh
```

### 3. Inside the Container
Once inside, ROS2 environment is already sourced. You can run:
```bash
# List all ROS2 packages
ros2 pkg list

# Launch UGV nodes (examples)
ros2 launch ugv_bringup ugv_bringup.launch.py
ros2 launch ugv_nav navigation.launch.py
ros2 launch ugv_slam slam.launch.py

# Run specific nodes
ros2 run ugv_base_node base_node
```

## Container Management

### Check if Container is Running
```bash
docker ps | grep ugv_rpi_ros_humble
```

### Stop the Container
```bash
docker stop ugv_rpi_ros_humble
```

### Remove the Container
```bash
docker stop ugv_rpi_ros_humble
docker rm ugv_rpi_ros_humble
```

### View Container Logs
```bash
docker logs ugv_rpi_ros_humble
```

### Execute Commands Without Entering
```bash
docker exec ugv_rpi_ros_humble bash -c "source /root/ugv_ws/install/setup.bash && ros2 pkg list"
```

## Available UGV Packages

- `ugv_base_node` - Base robot control
- `ugv_bringup` - Launch files for bringing up the robot
- `ugv_chat_ai` - AI chat interface
- `ugv_description` - URDF robot description
- `ugv_gazebo` - Gazebo simulation
- `ugv_interface` - ROS2 interfaces (msgs/srvs)
- `ugv_nav` - Navigation stack
- `ugv_slam` - SLAM capabilities
- `ugv_tools` - Utility tools
- `ugv_vision` - Vision processing
- `ugv_web_app` - Web interface
- `teb_local_planner` - TEB local planner (with libg2o fix applied)

## Troubleshooting

### Container won't start
```bash
# Check logs
docker logs ugv_rpi_ros_humble

# Remove and recreate
docker stop ugv_rpi_ros_humble
docker rm ugv_rpi_ros_humble
./start_ugv_service.sh
```

### Device not found (/dev/ttyACM0)
The start script will continue even if the device is not connected. Connect your hardware and restart the container.

### GUI applications don't work
Ensure X11 forwarding is set up correctly. Set DISPLAY variable before starting:
```bash
export DISPLAY=:0
./start_ugv_service.sh
```
