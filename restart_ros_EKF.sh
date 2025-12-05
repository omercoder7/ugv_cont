#!/bin/bash
# restart_ros_EKF.sh - Restart with EKF triple sensor fusion for best mapping
#
# This script sets up:
# 1. Robot bringup WITHOUT odom TF (rf2o publishes topic only)
# 2. EKF fusion (wheel + LiDAR + IMU) that publishes odom TF
# 3. SLAM using the fused odometry
#
# The EKF combines:
# - Wheel encoders: velocity (knows when robot is actually moving)
# - LiDAR odometry: position correction
# - IMU: orientation and angular velocity

CONTAINER_NAME="ugv_rpi_ros_humble"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== ROS2 EKF Fusion Restart Script ==="
echo ""

# Check container
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    exit 1
fi

# Show current state
echo "Current ROS2 nodes (before restart):"
docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | sort | uniq -c
echo ""

# Check for zombie processes
echo "Checking for zombie processes..."
ZOMBIES=$(docker exec ${CONTAINER_NAME} ps aux 2>/dev/null | grep "<defunct>" || true)
if [ -n "${ZOMBIES}" ]; then
    ZOMBIE_COUNT=$(echo "${ZOMBIES}" | wc -l)
    echo "  Found ${ZOMBIE_COUNT} zombies - restart required"
else
    echo "  No zombies found."
fi
echo ""

# Restart container
echo "Restarting container to clear all processes..."
docker restart ${CONTAINER_NAME}
sleep 5

# Verify clean state
echo ""
echo "Verifying clean state..."
ZOMBIES_AFTER=$(docker exec ${CONTAINER_NAME} bash -c "ps aux | grep -c '<defunct>'" 2>/dev/null || echo "0")
if [ "${ZOMBIES_AFTER}" -gt 0 ]; then
    echo "  WARNING: Still ${ZOMBIES_AFTER} zombies"
else
    echo "  Clean - no zombie processes!"
fi

# Copy configs
echo ""
echo "Copying configuration files..."
docker cp "${SCRIPT_DIR}/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml 2>/dev/null
docker cp "${SCRIPT_DIR}/ekf_wheel_lidar_imu.yaml" ${CONTAINER_NAME}:/tmp/ekf_wheel_lidar_imu.yaml 2>/dev/null
docker cp "${SCRIPT_DIR}/ekf_lidar_imu.yaml" ${CONTAINER_NAME}:/tmp/ekf_lidar_imu.yaml 2>/dev/null

# Start bringup WITHOUT odom TF (EKF will publish TF instead)
echo ""
echo "Starting robot bringup (WITHOUT odom TF - EKF will handle TF)..."
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
export UGV_MODEL=ugv_beast && \
export LDLIDAR_MODEL=ld19 && \
ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=false > /tmp/bringup.log 2>&1
"
sleep 6

# Check if bringup started
echo ""
echo "Checking bringup status..."
if docker exec ${CONTAINER_NAME} pgrep -f "bringup_lidar" > /dev/null 2>&1; then
    echo "  Bringup: RUNNING"
else
    echo "  Bringup: FAILED - check /tmp/bringup.log"
    exit 1
fi

# Verify sensors are publishing
echo ""
echo "Verifying sensor topics..."
for topic in "/odom/odom_raw" "/odom_rf2o" "/imu/data"; do
    HZ=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz ${topic} 2>/dev/null | head -1" 2>/dev/null)
    if [ -n "$HZ" ]; then
        echo "  ${topic}: OK"
    else
        echo "  ${topic}: WAITING..."
    fi
done

# Start EKF fusion node
echo ""
echo "Starting EKF fusion (wheel + LiDAR + IMU)..."
echo "  Inputs: /odom/odom_raw, /odom_rf2o, /imu/data"
echo "  Output: /odom_fused + odom->base_footprint TF"
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file /tmp/ekf_wheel_lidar_imu.yaml \
  -r /odometry/filtered:=/odom_fused > /tmp/ekf.log 2>&1
"
sleep 3

# Check EKF started
if docker exec ${CONTAINER_NAME} pgrep -f "ekf_node" > /dev/null 2>&1; then
    echo "  EKF: RUNNING"
else
    echo "  EKF: FAILED - check /tmp/ekf.log"
    docker exec ${CONTAINER_NAME} cat /tmp/ekf.log 2>/dev/null | tail -10
fi

# Check EKF is publishing TF
echo ""
echo "Checking EKF TF output..."
TF_CHECK=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep 'Translation'" 2>/dev/null)
if [ -n "$TF_CHECK" ]; then
    echo "  odom->base_footprint TF: OK"
    echo "  $TF_CHECK"
else
    echo "  odom->base_footprint TF: WAITING..."
fi

# Start SLAM
echo ""
echo "Starting SLAM with optimized config..."
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1
"
sleep 3

# Check if SLAM started
if docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
    echo "  SLAM: RUNNING"
else
    echo "  SLAM: FAILED - check /tmp/slam.log"
fi

# Final status
echo ""
echo "=== Final Status ==="
echo "Running nodes:"
docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | sort | uniq -c

echo ""
echo "=== Sensor Fusion Chain ==="
echo "  Wheel encoders (/odom/odom_raw) ─┐"
echo "  LiDAR odometry (/odom_rf2o)      ├─> EKF ─> /odom_fused + TF"
echo "  IMU (/imu/data)                  ─┘"
echo ""
echo "  SLAM uses odom->base_footprint TF from EKF"
echo ""

echo "=== Done ==="
echo "Run: ./rviz.sh slam-opt"
