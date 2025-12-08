#!/bin/bash
# Switch running UGV to EKF mode (restarts bringup with EKF)
# Use this when robot is already running but you want to switch to EKF

CONTAINER="ugv_rpi_ros_humble"

echo "=============================================="
echo "Switching to EKF Sensor Fusion Mode"
echo "=============================================="

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "ERROR: Container ${CONTAINER} is not running!"
    exit 1
fi

# Check current mode
echo "Checking current nodes..."
EKF_RUNNING=$(docker exec ${CONTAINER} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null | grep -c ekf_filter_node" 2>/dev/null || echo "0")

if [ "$EKF_RUNNING" != "0" ]; then
    echo "EKF is already running!"
    echo ""
    echo "Current EKF status:"
    docker exec ${CONTAINER} bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odometry/filtered --once 2>/dev/null | head -20"
    exit 0
fi

echo ""
echo "Current mode: Standard (no EKF)"
echo "Switching to EKF mode..."
echo ""

# Kill current bringup processes
echo "Stopping current bringup..."
docker exec ${CONTAINER} bash -c "
    pkill -f 'ugv_bringup' 2>/dev/null
    pkill -f 'ugv_driver' 2>/dev/null
    pkill -f 'base_node' 2>/dev/null
    pkill -f 'ldlidar' 2>/dev/null
    pkill -f 'complementary_filter' 2>/dev/null
" 2>/dev/null
sleep 2

# Launch EKF bringup in background
echo "Starting EKF bringup..."
docker exec -d ${CONTAINER} bash -c "
    source /opt/ros/humble/setup.bash && \
    source /home/ws/ugv_ws/install/setup.bash && \
    ros2 launch ugv_bringup bringup_imu_ekf.launch.py > /tmp/ekf_bringup.log 2>&1
"

# Wait for EKF to start
echo "Waiting for EKF node to start..."
for i in {1..10}; do
    sleep 1
    EKF_CHECK=$(docker exec ${CONTAINER} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null | grep -c ekf_filter_node" 2>/dev/null || echo "0")
    if [ "$EKF_CHECK" != "0" ]; then
        echo ""
        echo "SUCCESS! EKF mode is now active."
        echo ""
        echo "Nodes running:"
        docker exec ${CONTAINER} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null"
        echo ""
        echo "EKF publishes filtered odometry to: /odom"
        echo "  (remapped from /odometry/filtered)"
        exit 0
    fi
    echo -n "."
done

echo ""
echo "WARNING: EKF node did not start within timeout."
echo "Check logs: docker exec ${CONTAINER} cat /tmp/ekf_bringup.log"
exit 1
