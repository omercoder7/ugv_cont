#!/bin/bash
# ensure_slam.sh - Monitor and auto-restart SLAM if it stops updating
#
# Usage:
#   ./ensure_slam.sh           # Single check and fix
#   ./ensure_slam.sh --daemon  # Run in background, checking every 30 seconds
#   ./ensure_slam.sh --check   # Just check status, don't fix
#
# This script:
# 1. Checks if SLAM toolbox is running
# 2. Checks if /map topic is publishing
# 3. Checks if map->odom TF is being broadcast
# 4. Restarts SLAM if any check fails (with retries)

CONTAINER_NAME="ugv_rpi_ros_humble"
CHECK_INTERVAL=30  # seconds between checks in daemon mode
MAP_TIMEOUT=8      # seconds to wait for map message
MAX_RETRIES=3

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

check_container() {
    docker ps --filter "name=${CONTAINER_NAME}" --format '{{.Names}}' | grep -q "${CONTAINER_NAME}"
}

check_slam_process() {
    docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1
}

check_map_publishing() {
    # Try to get one map message within timeout
    local result
    result=$(docker exec ${CONTAINER_NAME} bash -c \
        "source /opt/ros/humble/setup.bash && timeout ${MAP_TIMEOUT} ros2 topic echo /map --once 2>/dev/null | grep -c 'frame_id: map'" 2>/dev/null || echo "0")
    [ "${result}" -gt 0 ]
}

check_map_tf() {
    # Check if map->odom transform is being published
    local result
    result=$(docker exec ${CONTAINER_NAME} bash -c \
        "source /opt/ros/humble/setup.bash && timeout 3 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -3" 2>/dev/null || echo "")
    echo "${result}" | grep -q "Translation"
}

check_prerequisites() {
    # Check if /scan is publishing (SLAM needs this)
    local scan_ok
    scan_ok=$(docker exec ${CONTAINER_NAME} bash -c \
        "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo /scan --once 2>/dev/null | head -1" 2>/dev/null || echo "")
    if [ -z "${scan_ok}" ]; then
        log_err "/scan not publishing - SLAM cannot work without LiDAR data"
        return 1
    fi

    # Check if odom->base_footprint TF exists (SLAM needs this)
    local tf_ok
    tf_ok=$(docker exec ${CONTAINER_NAME} bash -c \
        "source /opt/ros/humble/setup.bash && timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -3" 2>/dev/null || echo "")
    if ! echo "${tf_ok}" | grep -q "Translation"; then
        log_err "odom->base_footprint TF not available - bringup may not be running"
        return 1
    fi

    return 0
}

restart_slam() {
    log_warn "Restarting SLAM toolbox..."

    # Kill existing SLAM processes
    docker exec ${CONTAINER_NAME} pkill -f "slam_toolbox" 2>/dev/null
    sleep 2

    # Copy latest config (in case it was updated)
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    if [ -f "${SCRIPT_DIR}/config/slam_toolbox_optimized.yaml" ]; then
        docker cp "${SCRIPT_DIR}/config/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml 2>/dev/null
    fi

    # Start SLAM in background
    docker exec -d ${CONTAINER_NAME} bash -c \
        "source /opt/ros/humble/setup.bash && \
         source /root/ugv_ws/install/setup.bash && \
         ros2 launch slam_toolbox online_async_launch.py \
           use_sim_time:=false \
           slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1"

    # Wait for SLAM to initialize
    log "Waiting for SLAM to initialize..."
    sleep 5

    # Verify it started
    if check_slam_process && check_map_publishing; then
        log_ok "SLAM restarted successfully and publishing map!"
        return 0
    else
        log_err "SLAM restart failed"
        # Show last few lines of log for debugging
        docker exec ${CONTAINER_NAME} tail -10 /tmp/slam.log 2>/dev/null | head -5
        return 1
    fi
}

check_and_fix_slam() {
    # Check container first
    if ! check_container; then
        log_err "Container ${CONTAINER_NAME} not running!"
        return 1
    fi

    # Check prerequisites
    if ! check_prerequisites; then
        log_err "Prerequisites not met - cannot start SLAM"
        return 1
    fi

    # Check SLAM process
    if ! check_slam_process; then
        log_warn "SLAM process not running"

        # Try to restart with retries
        for attempt in $(seq 1 ${MAX_RETRIES}); do
            log "Restart attempt ${attempt}/${MAX_RETRIES}..."
            if restart_slam; then
                return 0
            fi
            sleep 2
        done

        log_err "Failed to start SLAM after ${MAX_RETRIES} attempts"
        return 1
    fi

    # Check map publishing
    if ! check_map_publishing; then
        log_warn "SLAM running but /map not publishing"

        # Check SLAM log for errors
        local slam_errors
        slam_errors=$(docker exec ${CONTAINER_NAME} tail -30 /tmp/slam.log 2>/dev/null | grep -i "error\|exception\|failed\|warn" | tail -5 || echo "")
        if [ -n "${slam_errors}" ]; then
            log_warn "Recent SLAM log issues:"
            echo "${slam_errors}"
        fi

        # Try restart
        for attempt in $(seq 1 ${MAX_RETRIES}); do
            log "Restart attempt ${attempt}/${MAX_RETRIES}..."
            if restart_slam; then
                return 0
            fi
            sleep 2
        done

        log_err "Failed to get SLAM publishing after ${MAX_RETRIES} attempts"
        return 1
    fi

    # Check map->odom TF
    if ! check_map_tf; then
        log_warn "SLAM running but map->odom TF not publishing"
        # This usually fixes itself, but let's restart if needed
        restart_slam
        return $?
    fi

    log_ok "SLAM OK - map publishing, TF working"
    return 0
}

status_only() {
    echo "=== SLAM Status Check ==="

    if ! check_container; then
        log_err "Container not running"
        return 1
    fi

    if check_slam_process; then
        log_ok "SLAM process: RUNNING"
    else
        log_err "SLAM process: NOT RUNNING"
    fi

    if check_map_publishing; then
        log_ok "/map topic: PUBLISHING"
    else
        log_err "/map topic: NOT PUBLISHING"
    fi

    if check_map_tf; then
        log_ok "map->odom TF: AVAILABLE"
    else
        log_err "map->odom TF: NOT AVAILABLE"
    fi

    # Show map info
    echo ""
    echo "Map info:"
    docker exec ${CONTAINER_NAME} bash -c \
        "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo /map --once 2>/dev/null | grep -E 'width|height|resolution'" 2>/dev/null || echo "  (no map data)"
}

# Main
case "$1" in
    --daemon)
        log "Starting SLAM monitor daemon (checking every ${CHECK_INTERVAL}s)"
        log "Press Ctrl+C to stop"
        echo ""

        while true; do
            check_and_fix_slam
            sleep ${CHECK_INTERVAL}
        done
        ;;
    --check|--status)
        # Just check status, don't fix
        status_only
        ;;
    *)
        # Default: single check and fix
        check_and_fix_slam
        exit $?
        ;;
esac
