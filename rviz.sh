#!/bin/bash
# Launch RViz2 for UGV visualization
# Usage: ./rviz.sh [slam-opt|slam|nav|lidar]

MODE="${1:-slam-opt}"
CONTAINER_NAME="ugv_rpi_ros_humble"

# Set up X11 forwarding
export DISPLAY=${DISPLAY:-:0}
xhost +local:docker 2>/dev/null

echo "=== Launching RViz2 (mode: $MODE) ==="

case $MODE in
    slam-opt|slam)
        echo "Starting SLAM visualization..."
        docker exec -e DISPLAY=$DISPLAY $CONTAINER_NAME bash -c "
            source /opt/ros/humble/setup.bash
            ros2 run rviz2 rviz2 -d /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml 2>/dev/null
        " 2>/dev/null &
        ;;
    nav)
        echo "Starting Navigation visualization..."
        docker exec -e DISPLAY=$DISPLAY $CONTAINER_NAME bash -c "
            source /opt/ros/humble/setup.bash
            ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz 2>/dev/null
        " 2>/dev/null &
        ;;
    lidar)
        echo "Starting LiDAR-only visualization..."
        docker exec -e DISPLAY=$DISPLAY $CONTAINER_NAME bash -c "
            source /opt/ros/humble/setup.bash
            ros2 run rviz2 rviz2 2>/dev/null
        " 2>/dev/null &
        ;;
    *)
        echo "Unknown mode: $MODE"
        echo "Usage: ./rviz.sh [slam-opt|slam|nav|lidar]"
        exit 1
        ;;
esac

echo "RViz2 starting in background..."
echo "If display issues occur, ensure X11 forwarding is configured."
