#!/bin/bash
# start_cartographer.sh - Start Google Cartographer for high-accuracy SLAM
#
# Cartographer provides better mapping than slam_toolbox by:
# - Using scan-to-submap matching (less reliant on odometry)
# - Global optimization with loop closure
# - More robust to odometry drift
#
# Usage:
#   ./start_cartographer.sh           # Start Cartographer
#   ./start_cartographer.sh --check   # Check if ready

CONTAINER_NAME="ugv_rpi_ros_humble"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check container is running
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    exit 1
fi

if [ "$1" == "--check" ]; then
    echo "=== Checking Cartographer Requirements ==="
    echo ""

    # Check if cartographer is installed
    CARTO=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list 2>/dev/null | grep cartographer_ros" 2>/dev/null)
    if [ -n "$CARTO" ]; then
        echo "1. Cartographer: INSTALLED"
    else
        echo "1. Cartographer: NOT INSTALLED"
        echo "   Run: docker exec ${CONTAINER_NAME} apt-get install -y ros-humble-cartographer-ros"
        exit 1
    fi

    # Check scan topic
    echo ""
    echo "2. LiDAR scan (/scan):"
    SCAN=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /scan 2>/dev/null | head -2" 2>/dev/null)
    if [ -n "$SCAN" ]; then
        echo "   $SCAN"
    else
        echo "   NOT AVAILABLE"
    fi

    # Check odom topic
    echo ""
    echo "3. Odometry (/odom):"
    ODOM=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /odom 2>/dev/null | head -2" 2>/dev/null)
    if [ -n "$ODOM" ]; then
        echo "   $ODOM"
    else
        echo "   NOT AVAILABLE"
    fi

    # Check TF
    echo ""
    echo "4. TF odom->base_footprint:"
    TF=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep Translation | head -1" 2>/dev/null)
    if [ -n "$TF" ]; then
        echo "   $TF"
    else
        echo "   NOT AVAILABLE"
    fi

    echo ""
    echo "=== Ready to run Cartographer ==="
    exit 0
fi

# Copy config to container
echo "Copying Cartographer config..."
docker cp "${SCRIPT_DIR}/cartographer_2d.lua" ${CONTAINER_NAME}:/tmp/cartographer_2d.lua

# Kill existing SLAM if running
if docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
    echo "Stopping existing slam_toolbox..."
    docker exec ${CONTAINER_NAME} pkill -f "slam_toolbox" 2>/dev/null
    sleep 2
fi

if docker exec ${CONTAINER_NAME} pgrep -f "cartographer_node" > /dev/null 2>&1; then
    echo "Stopping existing Cartographer..."
    docker exec ${CONTAINER_NAME} pkill -f "cartographer" 2>/dev/null
    sleep 2
fi

# Start Cartographer
echo "Starting Cartographer SLAM..."
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
ros2 run cartographer_ros cartographer_node \
  -configuration_directory /tmp \
  -configuration_basename cartographer_2d.lua > /tmp/cartographer.log 2>&1
"

sleep 3

# Check if started
if docker exec ${CONTAINER_NAME} pgrep -f "cartographer_node" > /dev/null 2>&1; then
    echo "Cartographer: RUNNING"
else
    echo "Cartographer: FAILED"
    echo ""
    echo "Log output:"
    docker exec ${CONTAINER_NAME} cat /tmp/cartographer.log 2>/dev/null | tail -20
    exit 1
fi

# Start occupancy grid node (publishes /map)
echo "Starting occupancy grid node..."
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  -resolution 0.03 \
  -publish_period_sec 1.0 > /tmp/occupancy_grid.log 2>&1
"

sleep 2

if docker exec ${CONTAINER_NAME} pgrep -f "occupancy_grid" > /dev/null 2>&1; then
    echo "Occupancy grid: RUNNING"
else
    echo "Occupancy grid: FAILED"
fi

echo ""
echo "=== Cartographer Started ==="
echo ""
echo "Topics:"
echo "  /map - Occupancy grid map"
echo "  /submap_list - Cartographer submaps"
echo "  /trajectory_node_list - Pose graph"
echo ""
echo "Use ./rviz.sh slam-carto to visualize"
