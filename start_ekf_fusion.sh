#!/bin/bash
# start_ekf_fusion.sh - Start robot_localization EKF with triple sensor fusion
#
# This script starts the EKF node that fuses:
# - Wheel encoders (/odom/odom_raw) - velocity
# - LiDAR odometry (/odom_rf2o) - position
# - IMU (/imu/data) - orientation
#
# Output: /odometry/filtered topic with fused odometry
#
# Usage:
#   ./start_ekf_fusion.sh         # Start EKF fusion
#   ./start_ekf_fusion.sh --check # Check sensor topics first

CONTAINER_NAME="ugv_rpi_ros_humble"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/ekf_wheel_lidar_imu.yaml"

# Check container is running
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Run ./restart_ros_DEBUG.sh first"
    exit 1
fi

# Check sensors if requested
if [ "$1" == "--check" ]; then
    echo "=== Checking Sensor Topics ==="
    echo ""

    echo "1. Wheel encoders (/odom/odom_raw):"
    WHEEL=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /odom/odom_raw 2>/dev/null | head -2" 2>/dev/null)
    if [ -n "$WHEEL" ]; then
        echo "   $WHEEL"
    else
        echo "   NOT AVAILABLE - wheel encoders not publishing"
    fi
    echo ""

    echo "2. LiDAR odometry (/odom_rf2o):"
    LIDAR=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /odom_rf2o 2>/dev/null | head -2" 2>/dev/null)
    if [ -n "$LIDAR" ]; then
        echo "   $LIDAR"
    else
        echo "   NOT AVAILABLE - rf2o not running"
    fi
    echo ""

    echo "3. IMU (/imu/data):"
    IMU=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /imu/data 2>/dev/null | head -2" 2>/dev/null)
    if [ -n "$IMU" ]; then
        echo "   $IMU"
    else
        echo "   NOT AVAILABLE - IMU not publishing"
    fi
    echo ""

    echo "=== Summary ==="
    echo "All three sensors should be publishing for best results."
    echo "Run without --check to start EKF fusion."
    exit 0
fi

# Copy config to container
echo "Copying EKF config to container..."
docker cp "${CONFIG_FILE}" ${CONTAINER_NAME}:/tmp/ekf_wheel_lidar_imu.yaml

# Check if EKF is already running
if docker exec ${CONTAINER_NAME} pgrep -f "ekf_node" > /dev/null 2>&1; then
    echo "EKF node is already running. Stopping it first..."
    docker exec ${CONTAINER_NAME} pkill -f "ekf_node" 2>/dev/null
    sleep 2
fi

# Start EKF node
echo "Starting EKF fusion node..."
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file /tmp/ekf_wheel_lidar_imu.yaml \
  -r /odometry/filtered:=/odom_fused > /tmp/ekf.log 2>&1
"

sleep 3

# Verify it started
if docker exec ${CONTAINER_NAME} pgrep -f "ekf_node" > /dev/null 2>&1; then
    echo "EKF node: RUNNING"
    echo ""
    echo "Output topic: /odom_fused"
    echo ""

    # Check if data is coming
    echo "Checking fused odometry output..."
    FUSED=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /odom_fused --once 2>/dev/null | grep -A3 'position:'" 2>/dev/null)
    if [ -n "$FUSED" ]; then
        echo "$FUSED"
    else
        echo "  Waiting for data... (check /tmp/ekf.log for errors)"
    fi
else
    echo "EKF node: FAILED to start"
    echo ""
    echo "Check logs:"
    docker exec ${CONTAINER_NAME} cat /tmp/ekf.log 2>/dev/null | tail -20
fi

echo ""
echo "=== Notes ==="
echo "- Fused odometry published to: /odom_fused"
echo "- To use with SLAM, configure slam_toolbox to use /odom_fused instead of /odom"
echo "- View logs: docker exec ${CONTAINER_NAME} cat /tmp/ekf.log"
