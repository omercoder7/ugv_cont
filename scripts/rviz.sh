#!/bin/bash
# rviz.sh - Launch RViz with SLAM and EKF sensor fusion
#
# Usage:
#   ./rviz.sh              # Launch RViz (assumes start_ros.sh already run)
#   ./rviz.sh new_map      # Restart container and start fresh map
#
# Examples:
#   ./start_ros.sh && ./rviz.sh           # Start ROS then launch RViz
#   ./rviz.sh new_map                     # Full restart with fresh map

CONTAINER_NAME="ugv_rpi_ros_humble"
NEW_MAP=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        new_map|new|--new-map)
            NEW_MAP=true
            ;;
        help|-h|--help)
            echo "UGV RViz Launcher (EKF Mode)"
            echo ""
            echo "Usage: ./rviz.sh [new_map]"
            echo ""
            echo "Options:"
            echo "  (none)    Launch RViz only (assumes start_ros.sh already run)"
            echo "  new_map   Restart container and start fresh map"
            echo ""
            echo "Examples:"
            echo "  ./start_ros.sh && ./rviz.sh   # Start ROS then launch RViz"
            echo "  ./rviz.sh new_map             # Full restart with fresh map"
            exit 0
            ;;
    esac
done

# Check if container is running
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Please start it using './start_ros.sh' first."
    exit 1
fi

# X11 setup - auto-detect SSH client IP for DISPLAY
if [ -z "${DISPLAY}" ]; then
    if [ -n "${SSH_CONNECTION}" ]; then
        SSH_CLIENT_IP=$(echo "${SSH_CONNECTION}" | awk '{print $1}')
        export DISPLAY="${SSH_CLIENT_IP}:0.0"
        echo "Auto-detected SSH client IP: ${SSH_CLIENT_IP}"
    else
        SSH_CLIENT_IP=$(who | grep -m1 "$(whoami)" | grep -oP '\(\K[0-9.]+(?=\))' 2>/dev/null)
        if [ -n "${SSH_CLIENT_IP}" ]; then
            export DISPLAY="${SSH_CLIENT_IP}:0.0"
            echo "Auto-detected client IP from 'who': ${SSH_CLIENT_IP}"
        else
            echo "Error: DISPLAY not set and could not auto-detect SSH client IP."
            echo "Please set DISPLAY manually: export DISPLAY=<your_ip>:0.0"
            exit 1
        fi
    fi
fi

echo "Using DISPLAY=${DISPLAY}"

# Copy xauth cookie to container for X11 authentication
DISPLAY_NUM=$(echo "${DISPLAY}" | sed 's/.*:\([0-9]*\).*/\1/')

XAUTH_COOKIE=$(xauth list 2>/dev/null | grep ":${DISPLAY_NUM}" | head -1 | awk '{print $3}')
if [ -z "${XAUTH_COOKIE}" ]; then
    XAUTH_COOKIE=$(xauth list "${DISPLAY}" 2>/dev/null | head -1 | awk '{print $3}')
fi

if [ -n "${XAUTH_COOKIE}" ]; then
    echo "Found X11 auth cookie for display :${DISPLAY_NUM}"
    docker exec ${CONTAINER_NAME} bash -c "
        touch /root/.Xauthority && \
        xauth add ${DISPLAY} MIT-MAGIC-COOKIE-1 ${XAUTH_COOKIE} 2>/dev/null
        xauth add localhost:${DISPLAY_NUM} MIT-MAGIC-COOKIE-1 ${XAUTH_COOKIE} 2>/dev/null
        xauth add localhost:${DISPLAY_NUM}.0 MIT-MAGIC-COOKIE-1 ${XAUTH_COOKIE} 2>/dev/null
    " 2>/dev/null
else
    echo "Warning: Could not find X11 auth cookie"
fi

xhost +local:docker 2>/dev/null
xhost + 2>/dev/null

# Get script and repo directories
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "${SCRIPT_DIR}")"

# Copy configs
docker cp "${REPO_DIR}/config/view_slam_2d.rviz" ${CONTAINER_NAME}:/tmp/view_slam_2d.rviz 2>/dev/null
docker cp "${REPO_DIR}/config/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml 2>/dev/null

# Handle new_map option - full restart for clean slate
if [ "${NEW_MAP}" = true ]; then
    echo ""
    echo "=== NEW MAP MODE - Full Restart ==="
    echo ""

    # Restart container to clear zombies
    echo "Restarting container..."
    docker restart ${CONTAINER_NAME}
    sleep 5

    # Reset ROS2 daemon
    docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 daemon stop && ros2 daemon start" 2>/dev/null

    # Re-copy configs after restart
    docker cp "${REPO_DIR}/config/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml
    docker cp "${REPO_DIR}/config/view_slam_2d.rviz" ${CONTAINER_NAME}:/tmp/view_slam_2d.rviz 2>/dev/null
    docker cp "${REPO_DIR}/launch/bringup_ekf_simple.launch.py" ${CONTAINER_NAME}:/root/ugv_ws/install/ugv_bringup/share/ugv_bringup/launch/ 2>/dev/null
    docker cp /home/ws/ugv_ws/src/ugv_main/ugv_bringup/param/ekf.yaml ${CONTAINER_NAME}:/root/ugv_ws/install/ugv_bringup/share/ugv_bringup/param/ekf.yaml 2>/dev/null

    # Start bringup with EKF
    echo "Starting robot bringup with EKF..."
    docker exec -d ${CONTAINER_NAME} /bin/bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ugv_ws/install/setup.bash && \
    export UGV_MODEL=ugv_beast && \
    export LDLIDAR_MODEL=ld19 && \
    ros2 launch ugv_bringup bringup_ekf_simple.launch.py > /tmp/bringup.log 2>&1
    "
    sleep 10

    # Check bringup
    if docker exec ${CONTAINER_NAME} pgrep -f "ros2" > /dev/null 2>&1; then
        echo "  Bringup: RUNNING"
    else
        echo "  Bringup: FAILED - check /tmp/bringup.log"
    fi

    # Start SLAM
    echo "Starting SLAM..."
    docker exec -d ${CONTAINER_NAME} /bin/bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ugv_ws/install/setup.bash && \
    ros2 launch slam_toolbox online_async_launch.py \
      use_sim_time:=false \
      slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1
    "
    sleep 3

    if docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
        echo "  SLAM: RUNNING"
    else
        echo "  SLAM: FAILED - check /tmp/slam.log"
    fi

    # Verify clean
    ZOMBIES=$(docker exec ${CONTAINER_NAME} ps aux 2>/dev/null | grep -c "<defunct>" || echo "0")
    if [ "${ZOMBIES}" -gt 0 ]; then
        echo "Warning: ${ZOMBIES} zombie processes"
    else
        echo "Clean start - no zombies!"
    fi
else
    # No new_map - just verify and launch RViz
    echo ""
    echo "=== Launching RViz (EKF Mode) ==="
    echo ""
    echo "NOTE: Run './start_ros.sh' first to start all nodes"
    echo "      Or use: ./rviz.sh new_map"
    echo ""

    # Just verify things are running
    echo "Checking system status..."

    RF2O_COUNT=$(docker exec ${CONTAINER_NAME} bash -c "ps aux | grep -c '[r]f2o_laser_odometry_node --ros-args'" 2>/dev/null || echo "0")
    EKF_RUNNING=$(docker exec ${CONTAINER_NAME} pgrep -f "ekf_node" > /dev/null 2>&1 && echo "yes" || echo "no")
    SLAM_RUNNING=$(docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1 && echo "yes" || echo "no")

    echo "  RF2O processes: ${RF2O_COUNT} (should be 1)"
    echo "  EKF running: ${EKF_RUNNING}"
    echo "  SLAM running: ${SLAM_RUNNING}"

    if [ "${RF2O_COUNT}" != "1" ] || [ "${EKF_RUNNING}" != "yes" ] || [ "${SLAM_RUNNING}" != "yes" ]; then
        echo ""
        echo "WARNING: System not fully running!"
        echo "Run: ./start_ros.sh"
        echo ""
    fi
fi

echo ""
echo "Launching RViz..."

# Detect if we have a TTY
DOCKER_FLAGS="-i"
if [ -t 0 ]; then
    DOCKER_FLAGS="-it"
fi

# Copy host's Xauthority file to container for X11 auth
if [ -f "${HOME}/.Xauthority" ]; then
    docker cp "${HOME}/.Xauthority" ${CONTAINER_NAME}:/tmp/.Xauthority 2>/dev/null
fi

# Launch RViz with software rendering for X11 forwarding
docker exec ${DOCKER_FLAGS} \
    -e DISPLAY=${DISPLAY} \
    -e XAUTHORITY=/tmp/.Xauthority \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e QT_X11_NO_MITSHM=1 \
    ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
export UGV_MODEL=ugv_beast && \
export LDLIDAR_MODEL=ld19 && \
rviz2 -d /tmp/view_slam_2d.rviz
"
