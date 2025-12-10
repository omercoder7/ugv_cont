#!/bin/bash
# Start UGV with EKF sensor fusion mode
# This uses IMU + laser odometry fusion for better localization

CONTAINER="ugv_rpi_ros_humble"

echo "=============================================="
echo "Starting UGV with EKF Sensor Fusion Mode"
echo "=============================================="
echo ""
echo "EKF Configuration:"
echo "  - RF2O laser odometry: x, y position + vx, vy velocity"
echo "  - IMU: yaw orientation + yaw_rate (authoritative)"
echo "  - No IMU acceleration (prevents drift)"
echo "  - Differential mode for position (reduces drift)"
echo "  - 50Hz output frequency"
echo ""

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "ERROR: Container ${CONTAINER} is not running!"
    echo "Please start the container first."
    exit 1
fi

# Kill any existing ROS nodes (clean slate)
echo "Stopping existing ROS nodes..."
docker exec ${CONTAINER} bash -c "pkill -f 'ros2|python3.*ros' 2>/dev/null || true"
sleep 2

# Source ROS and launch EKF bringup
echo ""
echo "Launching EKF bringup..."
echo "(Press Ctrl+C to stop)"
echo ""

docker exec -it ${CONTAINER} bash -c "
    source /opt/ros/humble/setup.bash && \
    source /home/ws/ugv_ws/install/setup.bash && \
    ros2 launch ugv_bringup bringup_imu_ekf.launch.py
"
