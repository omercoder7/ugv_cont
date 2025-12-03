#!/bin/bash
# restart_ros_DEBUG.sh - Kill all ROS2 nodes and restart cleanly
# Use this when you have duplicate nodes causing issues

CONTAINER_NAME="ugv_rpi_ros_humble"

echo "=== ROS2 Debug Restart Script ==="
echo ""

# Check container
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    exit 1
fi

# Show current state
echo "Current ROS2 nodes (before cleanup):"
docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | sort | uniq -c
echo ""

# Kill ALL processes
echo "Killing all ROS2 processes..."
docker exec ${CONTAINER_NAME} bash -c "killall -9 python3 2>/dev/null; killall -9 rviz2 2>/dev/null" 2>/dev/null
sleep 1

# Double-check with specific process names
docker exec ${CONTAINER_NAME} pkill -9 -f "slam_toolbox" 2>/dev/null
docker exec ${CONTAINER_NAME} pkill -9 -f "rf2o" 2>/dev/null
docker exec ${CONTAINER_NAME} pkill -9 -f "ldlidar" 2>/dev/null
docker exec ${CONTAINER_NAME} pkill -9 -f "base_node" 2>/dev/null
docker exec ${CONTAINER_NAME} pkill -9 -f "robot_state" 2>/dev/null
docker exec ${CONTAINER_NAME} pkill -9 -f "joint_state" 2>/dev/null
docker exec ${CONTAINER_NAME} pkill -9 -f "ugv_driver" 2>/dev/null
docker exec ${CONTAINER_NAME} pkill -9 -f "ugv_bringup" 2>/dev/null
docker exec ${CONTAINER_NAME} pkill -9 -f "ros2" 2>/dev/null
sleep 2

# Verify cleanup
echo ""
echo "Nodes after cleanup:"
NODES=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | wc -l)
echo "  Count: ${NODES}"

if [ "${NODES}" -gt 0 ]; then
    echo "  Warning: Some nodes still running:"
    docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null"
    echo ""
    echo "Attempting more aggressive cleanup..."
    docker exec ${CONTAINER_NAME} bash -c "ps aux | grep -E 'ros2|python3' | grep -v grep | awk '{print \$2}' | xargs -r kill -9" 2>/dev/null
    sleep 2
fi

# Start bringup
echo ""
echo "Starting robot bringup..."
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
export UGV_MODEL=ugv_beast && \
export LDLIDAR_MODEL=ld19 && \
ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true 2>&1 | tee /tmp/bringup.log
"
sleep 6

# Check if bringup started
echo ""
echo "Checking bringup status..."
if docker exec ${CONTAINER_NAME} pgrep -f "bringup_lidar" > /dev/null 2>&1; then
    echo "  Bringup: RUNNING"
else
    echo "  Bringup: FAILED - check /tmp/bringup.log inside container"
fi

# Start SLAM
echo ""
echo "Starting SLAM..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker cp "${SCRIPT_DIR}/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml 2>/dev/null

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
