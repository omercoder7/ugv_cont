#!/bin/bash
# start_ros.sh - Start container and ROS2 nodes cleanly with ROBUST SLAM startup
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
SLAM_MAX_RETRIES=3
SLAM_VERIFY_TIMEOUT=10

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "[$(date '+%H:%M:%S')] $1"
}

log_ok() {
    echo -e "[$(date '+%H:%M:%S')] ${GREEN}✓ $1${NC}"
}

log_warn() {
    echo -e "[$(date '+%H:%M:%S')] ${YELLOW}⚠ $1${NC}"
}

log_err() {
    echo -e "[$(date '+%H:%M:%S')] ${RED}✗ $1${NC}"
}

# Parse arguments
for arg in "$@"; do
    case $arg in
        --ekf)
            USE_EKF=true
            shift
            ;;
    esac
done

echo "=== ROS2 Start Script (Robust SLAM) ==="
if [ "$USE_EKF" = true ]; then
    echo "Mode: EKF (IMU + Laser Odometry fusion)"
else
    echo "Mode: Standard (Laser Odometry only)"
fi
echo ""

# Check container
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    log_err "Container '${CONTAINER_NAME}' is not running."
    exit 1
fi

# Show current state
log "Current ROS2 nodes (before restart):"
docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | sort | uniq -c
echo ""

# Check for zombie processes
log "Checking for zombie processes..."
ZOMBIES=$(docker exec ${CONTAINER_NAME} ps aux 2>/dev/null | grep "<defunct>" || true)
if [ -n "${ZOMBIES}" ]; then
    ZOMBIE_COUNT=$(echo "${ZOMBIES}" | wc -l)
    log_warn "Found ${ZOMBIE_COUNT} zombie processes - container restart required"
else
    log_ok "No zombies found"
fi
echo ""

# Restart container to clear ALL processes including zombies
log "Restarting container to clear all processes..."
docker restart ${CONTAINER_NAME}
sleep 5

# Reset ROS2 daemon inside container to clear DDS discovery cache
log "Resetting ROS2 daemon (clears DDS cache)..."
docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 daemon stop && ros2 daemon start" 2>/dev/null
sleep 1

# Verify clean state
ZOMBIES_AFTER=$(docker exec ${CONTAINER_NAME} bash -c "ps aux | grep '<defunct>' | grep -v grep | wc -l" 2>/dev/null || echo "0")
if [ "${ZOMBIES_AFTER}" -gt 0 ]; then
    log_warn "Still ${ZOMBIES_AFTER} zombies (unexpected)"
else
    log_ok "Container is clean - no zombie processes"
fi

# Copy configs
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "${SCRIPT_DIR}")"
docker cp "${REPO_DIR}/config/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml 2>/dev/null

# ============================================================================
# START BRINGUP
# ============================================================================
echo ""
log "=== Starting Robot Bringup ==="

if [ "$USE_EKF" = true ]; then
    log "Starting EKF mode bringup..."
    # Copy EKF launch file and config to container
    if [ -f "${REPO_DIR}/launch/bringup_ekf_simple.launch.py" ]; then
        docker cp "${REPO_DIR}/launch/bringup_ekf_simple.launch.py" ${CONTAINER_NAME}:/root/ugv_ws/install/ugv_bringup/share/ugv_bringup/launch/
    fi
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
    BRINGUP_WAIT=12
else
    log "Starting standard mode bringup..."
    docker exec -d ${CONTAINER_NAME} /bin/bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ugv_ws/install/setup.bash && \
    export UGV_MODEL=ugv_beast && \
    export LDLIDAR_MODEL=ld19 && \
    ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true > /tmp/bringup.log 2>&1
    "
    BRINGUP_WAIT=8
fi

# Wait for bringup with progress
log "Waiting for bringup to initialize (${BRINGUP_WAIT}s)..."
for i in $(seq 1 ${BRINGUP_WAIT}); do
    echo -ne "\r  Progress: ${i}/${BRINGUP_WAIT}s"
    sleep 1
done
echo ""

# Verify bringup is running
if docker exec ${CONTAINER_NAME} pgrep -f "ugv_bringup\|ldlidar" > /dev/null 2>&1; then
    log_ok "Bringup process is running"
else
    log_err "Bringup FAILED - check /tmp/bringup.log"
    exit 1
fi

# ============================================================================
# VERIFY TF TREE IS READY (Critical for SLAM)
# ============================================================================
echo ""
log "=== Verifying TF Tree ==="

# Wait for odom->base_footprint transform (required by SLAM)
TF_READY=false
for i in $(seq 1 10); do
    TF_CHECK=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -3" 2>/dev/null || echo "")
    if echo "${TF_CHECK}" | grep -q "Translation"; then
        TF_READY=true
        break
    fi
    echo -ne "\r  Waiting for TF: ${i}/10s"
    sleep 1
done
echo ""

if [ "${TF_READY}" = true ]; then
    log_ok "TF tree is ready (odom->base_footprint)"
else
    log_warn "TF tree not fully ready - SLAM may have issues"
fi

# Verify /scan topic is publishing
SCAN_CHECK=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo /scan --once 2>/dev/null | head -1" 2>/dev/null || echo "")
if [ -n "${SCAN_CHECK}" ]; then
    log_ok "/scan topic is publishing"
else
    log_warn "/scan topic not publishing - check LiDAR"
fi

# ============================================================================
# START SLAM WITH RETRY MECHANISM
# ============================================================================
echo ""
log "=== Starting SLAM Toolbox ==="

start_slam() {
    docker exec ${CONTAINER_NAME} pkill -f "slam_toolbox" 2>/dev/null
    sleep 2

    docker exec -d ${CONTAINER_NAME} /bin/bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ugv_ws/install/setup.bash && \
    ros2 launch slam_toolbox online_async_launch.py \
      use_sim_time:=false \
      slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1
    "
}

verify_slam() {
    # Check 1: SLAM process is running
    if ! docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
        return 1
    fi

    # Check 2: /map topic is publishing (most important!)
    MAP_CHECK=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout ${SLAM_VERIFY_TIMEOUT} ros2 topic echo /map --once 2>/dev/null | grep -c 'frame_id: map'" 2>/dev/null || echo "0")
    if [ "${MAP_CHECK}" -lt 1 ]; then
        return 2
    fi

    # Check 3: map->odom transform is being published
    TF_MAP=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -3" 2>/dev/null || echo "")
    if ! echo "${TF_MAP}" | grep -q "Translation"; then
        return 3
    fi

    return 0
}

# Try to start SLAM with retries
SLAM_SUCCESS=false
for attempt in $(seq 1 ${SLAM_MAX_RETRIES}); do
    log "SLAM startup attempt ${attempt}/${SLAM_MAX_RETRIES}..."

    start_slam

    # Wait for SLAM to initialize
    log "  Waiting for SLAM to initialize..."
    sleep 5

    # Verify SLAM is working
    verify_slam
    SLAM_STATUS=$?

    case ${SLAM_STATUS} in
        0)
            log_ok "SLAM is running and publishing map!"
            SLAM_SUCCESS=true
            break
            ;;
        1)
            log_warn "  SLAM process not running"
            ;;
        2)
            log_warn "  SLAM running but /map not publishing"
            # Check SLAM log for errors
            SLAM_ERRORS=$(docker exec ${CONTAINER_NAME} tail -20 /tmp/slam.log 2>/dev/null | grep -i "error\|exception\|failed" || echo "")
            if [ -n "${SLAM_ERRORS}" ]; then
                log_warn "  SLAM errors found:"
                echo "${SLAM_ERRORS}" | head -5
            fi
            ;;
        3)
            log_warn "  SLAM running but map->odom TF not publishing"
            ;;
    esac

    if [ ${attempt} -lt ${SLAM_MAX_RETRIES} ]; then
        log "  Retrying in 3 seconds..."
        sleep 3
    fi
done

if [ "${SLAM_SUCCESS}" = false ]; then
    log_err "SLAM failed to start after ${SLAM_MAX_RETRIES} attempts!"
    log_err "Check /tmp/slam.log inside container for details"
    log "Continuing anyway - robot will work but mapping won't update"
fi

# ============================================================================
# VERIFY EKF (if enabled)
# ============================================================================
if [ "$USE_EKF" = true ]; then
    echo ""
    log "=== Verifying EKF ==="

    if docker exec ${CONTAINER_NAME} pgrep -f "ekf_node" > /dev/null 2>&1; then
        EKF_RATE=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /odom --window 3 2>&1 | grep 'average rate' | head -1 | awk '{print \$3}'" 2>/dev/null || echo "0")
        log_ok "EKF running at ${EKF_RATE} Hz"
    else
        log_warn "EKF not running - starting manually..."
        docker exec -d ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash && \
        source /root/ugv_ws/install/setup.bash && \
        ros2 run robot_localization ekf_node \
          --ros-args \
          --params-file /root/ugv_ws/install/ugv_bringup/share/ugv_bringup/param/ekf.yaml \
          -r /odometry/filtered:=/odom > /tmp/ekf.log 2>&1
        "
        sleep 3
    fi
fi

# ============================================================================
# FINAL STATUS
# ============================================================================
echo ""
log "=== Final Status ==="

# Check all key components
BRINGUP_OK=false
SCAN_OK=false
ODOM_OK=false
SLAM_OK=false
MAP_OK=false

# Bringup
if docker exec ${CONTAINER_NAME} pgrep -f "ugv_bringup\|ldlidar" > /dev/null 2>&1; then
    BRINGUP_OK=true
    log_ok "Bringup: RUNNING"
else
    log_err "Bringup: NOT RUNNING"
fi

# Scan topic
if docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null | grep -q '/scan'"; then
    SCAN_OK=true
    log_ok "/scan: AVAILABLE"
else
    log_err "/scan: NOT AVAILABLE"
fi

# Odometry
ODOM_TEST=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo /odom --once 2>/dev/null | grep -c 'position'" 2>/dev/null || echo "0")
if [ "${ODOM_TEST}" -gt 0 ]; then
    ODOM_OK=true
    log_ok "/odom: PUBLISHING"
else
    log_err "/odom: NOT PUBLISHING"
fi

# SLAM process
if docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
    SLAM_OK=true
    log_ok "SLAM: RUNNING"
else
    log_err "SLAM: NOT RUNNING"
fi

# Map topic (most important for mapping)
MAP_TEST=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /map --once 2>/dev/null | grep -c 'frame_id: map'" 2>/dev/null || echo "0")
if [ "${MAP_TEST}" -gt 0 ]; then
    MAP_OK=true
    log_ok "/map: PUBLISHING (mapping will work!)"
else
    log_err "/map: NOT PUBLISHING (mapping will NOT update!)"
fi

echo ""
if [ "${BRINGUP_OK}" = true ] && [ "${SCAN_OK}" = true ] && [ "${ODOM_OK}" = true ] && [ "${SLAM_OK}" = true ] && [ "${MAP_OK}" = true ]; then
    log_ok "=== ALL SYSTEMS GO! ==="
else
    log_warn "=== Some components have issues - check above ==="
fi

echo ""
log "=== Done ==="
