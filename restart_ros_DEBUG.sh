#!/bin/bash
# Restart ROS2 nodes in the UGV container
# Usage: ./restart_ros_DEBUG.sh

CONTAINER_NAME="ugv_rpi_ros_humble"

echo "=== Stopping existing ROS nodes ==="
docker exec $CONTAINER_NAME bash -c "pkill -f ros2 2>/dev/null; pkill -f slam_toolbox 2>/dev/null; pkill -f rf2o 2>/dev/null" 2>/dev/null
sleep 2

echo "=== Starting UGV bringup ==="
docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && ros2 launch ugv_bringup bringup.launch.py 2>&1 | tee /tmp/bringup.log" 2>/dev/null
sleep 3

echo "=== Starting SLAM Toolbox ==="
docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && ros2 launch slam_toolbox online_async_launch.py 2>&1 | tee /tmp/slam.log" 2>/dev/null
sleep 2

echo "=== Starting RF2O Odometry ==="
docker exec -d $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py 2>&1 | tee /tmp/rf2o.log" 2>/dev/null
sleep 2

echo ""
echo "=== Final Status ==="
echo "Running nodes:"
docker exec $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | sort | uniq -c

echo ""
echo "=== Duplicate Check ==="
DUPES=$(docker exec $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | sort | uniq -c | grep -v "^\s*1 ")
if [ -n "$DUPES" ]; then
    echo "  WARNING: Duplicate nodes detected:"
    echo "$DUPES"
else
    echo "  OK: No duplicate nodes"
fi

echo ""
echo "=== Done ==="
echo "You can now run: ./rviz.sh slam-opt"
