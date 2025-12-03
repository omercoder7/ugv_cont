#!/bin/bash
# restart_ros_DEBUG.sh - Restart container and ROS2 nodes cleanly
# Use this when you have duplicate nodes or zombie processes causing issues
#
# IMPORTANT: This script restarts the Docker container to clear zombie processes.
# Killing processes alone does NOT remove zombies - only container restart does.

CONTAINER_NAME="ugv_rpi_ros_humble"

echo "=== ROS2 Debug Restart Script ==="
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
    echo "  Found zombie processes:"
    echo "${ZOMBIES}" | head -10
    ZOMBIE_COUNT=$(echo "${ZOMBIES}" | wc -l)
    echo "  Total: ${ZOMBIE_COUNT} zombies"
    echo ""
    echo "  NOTE: Zombies cause odometry drift (Eigensolver errors)."
    echo "  Container restart is required to clear them."
else
    echo "  No zombies found."
fi
echo ""

# Restart container to clear ALL processes including zombies
echo "Restarting container to clear all processes and zombies..."
docker restart ${CONTAINER_NAME}
sleep 5

# Verify zombies are cleared
echo ""
echo "Verifying clean state..."
ZOMBIES_AFTER=$(docker exec ${CONTAINER_NAME} bash -c "ps aux | grep -c '<defunct>'" 2>/dev/null || echo "0")
if [ "${ZOMBIES_AFTER}" -gt 0 ]; then
    echo "  WARNING: Still ${ZOMBIES_AFTER} zombies (unexpected)"
else
    echo "  Clean - no zombie processes!"
fi

# Copy configs
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker cp "${SCRIPT_DIR}/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml 2>/dev/null
docker cp "${SCRIPT_DIR}/ekf_lidar_imu.yaml" ${CONTAINER_NAME}:/tmp/ekf_lidar_imu.yaml 2>/dev/null

# Start bringup (without TF publishing - EKF will handle it)
echo ""
echo "Starting robot bringup..."
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
export UGV_MODEL=ugv_beast && \
export LDLIDAR_MODEL=ld19 && \
ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=false 2>&1 | tee /tmp/bringup.log
"
sleep 6

# Start EKF for sensor fusion (fuses laser odometry + IMU)
echo ""
echo "Starting EKF sensor fusion..."
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
ros2 run robot_localization ekf_node --ros-args --params-file /tmp/ekf_lidar_imu.yaml 2>&1 | tee /tmp/ekf.log
"
sleep 2

# Check if bringup started
echo ""
echo "Checking bringup status..."
if docker exec ${CONTAINER_NAME} pgrep -f "bringup_lidar" > /dev/null 2>&1; then
    echo "  Bringup: RUNNING"
else
    echo "  Bringup: FAILED - check /tmp/bringup.log inside container"
fi

# Check odometry
echo ""
echo "Checking odometry..."
ODOM=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /odom --once 2>/dev/null | grep -A3 'position:'" 2>/dev/null)
if [ -n "${ODOM}" ]; then
    echo "${ODOM}"
else
    echo "  Waiting for odometry topic..."
fi

# Start SLAM
echo ""
echo "Starting SLAM..."
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=/tmp/slam_toolbox_optimized.yaml 2>&1 | tee /tmp/slam.log
"
sleep 3

# Check if SLAM started
if docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
    echo "  SLAM: RUNNING"
else
    echo "  SLAM: FAILED - check /tmp/slam.log inside container"
fi

# Final status
echo ""
echo "=== Final Status ==="
echo "Running nodes:"
docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | sort | uniq -c

echo ""
echo "=== Duplicate Check ==="
DUPS=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | sort | uniq -c | awk '$1 > 1 {print}')
if [ -z "${DUPS}" ]; then
    echo "  No duplicates found - system is clean!"
else
    echo "  WARNING: Duplicate nodes detected:"
    echo "${DUPS}"
fi

echo ""
echo "=== Done ==="
echo "You can now run: ./rviz.sh slam-opt"
