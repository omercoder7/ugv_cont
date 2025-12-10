#!/bin/bash
# start_ros.sh - Start container and ROS2 nodes cleanly
# Use this to start the robot with proper sensor fusion
#
# IMPORTANT: This script restarts the Docker container to clear zombie processes.
# Killing processes alone does NOT remove zombies - only container restart does.
#
# Usage:
#   ./start_ros.sh         # Standard bringup (no EKF)
#   ./start_ros.sh --ekf   # EKF mode (sensor fusion) - RECOMMENDED

CONTAINER_NAME="ugv_rpi_ros_humble"
USE_EKF=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        --ekf)
            USE_EKF=true
            shift
            ;;
    esac
done

echo "=== ROS2 Start Script ==="
if [ "$USE_EKF" = true ]; then
    echo "Mode: EKF (IMU + Laser Odometry fusion)"
else
    echo "Mode: Standard (Laser Odometry only)"
fi
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

# Reset ROS2 daemon inside container to clear DDS discovery cache
echo "Resetting ROS2 daemon (clears DDS cache)..."
docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 daemon stop && ros2 daemon start" 2>/dev/null

# Verify zombies are cleared
echo ""
echo "Verifying clean state..."
ZOMBIES_AFTER=$(docker exec ${CONTAINER_NAME} bash -c "ps aux | grep '<defunct>' | grep -v grep | wc -l" 2>/dev/null || echo "0")
if [ "${ZOMBIES_AFTER}" -gt 0 ]; then
    echo "  WARNING: Still ${ZOMBIES_AFTER} zombies (unexpected)"
else
    echo "  Clean - no zombie processes!"
fi

# Copy configs
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "${SCRIPT_DIR}")"
docker cp "${REPO_DIR}/config/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml 2>/dev/null

# Start bringup
echo ""
if [ "$USE_EKF" = true ]; then
    echo "Starting robot bringup (EKF mode)..."
    # Copy EKF launch file and config to container
    if [ -f "${REPO_DIR}/launch/bringup_ekf_simple.launch.py" ]; then
        docker cp "${REPO_DIR}/launch/bringup_ekf_simple.launch.py" ${CONTAINER_NAME}:/root/ugv_ws/install/ugv_bringup/share/ugv_bringup/launch/
    fi
    # Copy optimized EKF config (velocity-only mode for zero-covariance RF2O)
    if [ -f "/home/ws/ugv_ws/src/ugv_main/ugv_bringup/param/ekf.yaml" ]; then
        docker cp /home/ws/ugv_ws/src/ugv_main/ugv_bringup/param/ekf.yaml ${CONTAINER_NAME}:/root/ugv_ws/install/ugv_bringup/share/ugv_bringup/param/ekf.yaml
    fi
    docker exec -d ${CONTAINER_NAME} /bin/bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ugv_ws/install/setup.bash && \
    export UGV_MODEL=ugv_beast && \
    export LDLIDAR_MODEL=ld19 && \
    ros2 launch ugv_bringup bringup_ekf_simple.launch.py > /tmp/bringup.log 2>&1
    "
    sleep 10  # EKF needs more time to initialize
else
    echo "Starting robot bringup (standard mode)..."
    if [ -x "${SCRIPT_DIR}/ensure_bringup.sh" ]; then
        "${SCRIPT_DIR}/ensure_bringup.sh" --wait
    else
        docker exec -d ${CONTAINER_NAME} /bin/bash -c "
        source /opt/ros/humble/setup.bash && \
        source /root/ugv_ws/install/setup.bash && \
        export UGV_MODEL=ugv_beast && \
        export LDLIDAR_MODEL=ld19 && \
        ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true > /tmp/bringup.log 2>&1
        "
        sleep 6
    fi
fi

# Check if bringup started
echo ""
echo "Checking bringup status..."
if docker exec ${CONTAINER_NAME} pgrep -f "ugv_bringup" > /dev/null 2>&1; then
    echo "  Bringup: RUNNING"
else
    echo "  Bringup: FAILED - check /tmp/bringup.log inside container"
fi

# Check EKF if in EKF mode - verify it's actually running and outputting
if [ "$USE_EKF" = true ]; then
    EKF_RUNNING=false

    # Check if EKF process exists
    if docker exec ${CONTAINER_NAME} pgrep -f "ekf_node" > /dev/null 2>&1; then
        # Verify EKF is outputting at proper rate (should be ~10Hz, not 1Hz)
        EKF_RATE=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /odom --window 3 2>&1 | grep 'average rate' | head -1 | awk '{print \$3}'" 2>/dev/null || echo "0")
        EKF_RATE_INT=$(echo "${EKF_RATE}" | cut -d'.' -f1)

        if [ -n "${EKF_RATE_INT}" ] && [ "${EKF_RATE_INT}" -ge 8 ]; then
            echo "  EKF: RUNNING at ${EKF_RATE} Hz ✓"
            EKF_RUNNING=true
        else
            echo "  EKF: Running but output rate is ${EKF_RATE} Hz (expected ~10Hz)"
            echo "  Checking RF2O..."
            RF2O_RATE=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /odom_rf2o --window 3 2>&1 | grep 'average rate' | head -1 | awk '{print \$3}'" 2>/dev/null || echo "0")
            if [ -z "${RF2O_RATE}" ] || [ "${RF2O_RATE}" = "0" ]; then
                echo "  RF2O not publishing! EKF has no input."
            else
                echo "  RF2O publishing at ${RF2O_RATE} Hz"
                EKF_RUNNING=true
            fi
        fi
    else
        echo "  EKF: NOT RUNNING - attempting to start..."
        docker exec -d ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash && \
        source /root/ugv_ws/install/setup.bash && \
        ros2 run robot_localization ekf_node \
          --ros-args \
          --params-file /root/ugv_ws/install/ugv_bringup/share/ugv_bringup/param/ekf.yaml \
          -r /odometry/filtered:=/odom > /tmp/ekf.log 2>&1
        "
        sleep 3
        if docker exec ${CONTAINER_NAME} pgrep -f "ekf_node" > /dev/null 2>&1; then
            echo "  EKF: STARTED manually"
            EKF_RUNNING=true
        else
            echo "  EKF: FAILED TO START - check /tmp/ekf.log"
        fi
    fi
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
  slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1
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
echo "=== Duplicate Process Check ==="
# NOTE: ros2 node list can show duplicates due to DDS cache bug even with 1 process
# So we check actual PROCESSES, not node names
RF2O_PROCS=$(docker exec ${CONTAINER_NAME} bash -c "ps aux | grep -c '[r]f2o_laser_odometry_node --ros-args'" 2>/dev/null || echo "0")
SLAM_PROCS=$(docker exec ${CONTAINER_NAME} bash -c "ps aux | grep -c '[a]sync_slam_toolbox_node'" 2>/dev/null || echo "0")
EKF_PROCS=$(docker exec ${CONTAINER_NAME} bash -c "ps aux | grep -c '[e]kf_node'" 2>/dev/null || echo "0")

DUPLICATES_FOUND=0
if [ "${RF2O_PROCS}" -gt 1 ]; then
    echo "  WARNING: ${RF2O_PROCS} RF2O processes (should be 1)"
    DUPLICATES_FOUND=1
fi
if [ "${SLAM_PROCS}" -gt 1 ]; then
    echo "  WARNING: ${SLAM_PROCS} SLAM processes (should be 1)"
    DUPLICATES_FOUND=1
fi
if [ "${EKF_PROCS}" -gt 1 ]; then
    echo "  WARNING: ${EKF_PROCS} EKF processes (should be 1)"
    DUPLICATES_FOUND=1
fi

if [ "${DUPLICATES_FOUND}" -eq 0 ]; then
    echo "  No duplicate processes - system is clean!"
    echo "  (Note: ros2 node list may show duplicates due to DDS cache - this is normal)"
fi

echo ""
echo "=== Done ==="
