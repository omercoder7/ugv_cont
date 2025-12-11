#!/bin/bash
# ensure_slam.sh - Monitor and auto-restart SLAM if it stops updating
#
# Usage: ./ensure_slam.sh [--daemon]
#   --daemon: Run in background, checking every 30 seconds
#
# This script:
# 1. Checks if SLAM toolbox is running
# 2. Checks if /map topic is publishing
# 3. Restarts SLAM if either check fails

CONTAINER_NAME="ugv_rpi_ros_humble"
CHECK_INTERVAL=30  # seconds between checks in daemon mode
MAP_TIMEOUT=5      # seconds to wait for map message

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "[$(date '+%H:%M:%S')] $1"
}

check_container() {
    docker ps --filter "name=${CONTAINER_NAME}" --format '{{.Names}}' | grep -q "${CONTAINER_NAME}"
}

check_slam_process() {
    docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1
}

check_map_publishing() {
    # Try to get one map message within timeout
    timeout ${MAP_TIMEOUT} docker exec ${CONTAINER_NAME} bash -c \
        "source /opt/ros/humble/setup.bash && ros2 topic echo /map --once 2>/dev/null" | grep -q "frame_id: map"
}

restart_slam() {
    log "${YELLOW}Restarting SLAM toolbox...${NC}"

    # Kill existing SLAM processes
    docker exec ${CONTAINER_NAME} pkill -f "slam_toolbox" 2>/dev/null
    sleep 2

    # Start SLAM in background
    docker exec -d ${CONTAINER_NAME} bash -c \
        "source /opt/ros/humble/setup.bash && \
         source /root/ugv_ws/install/setup.bash && \
         ros2 launch slam_toolbox online_async_launch.py > /tmp/slam.log 2>&1"

    # Wait for SLAM to initialize
    sleep 5

    # Verify it started
    if check_slam_process && check_map_publishing; then
        log "${GREEN}SLAM restarted successfully${NC}"
        return 0
    else
        log "${RED}SLAM restart failed!${NC}"
        return 1
    fi
}

check_and_fix_slam() {
    # Check container first
    if ! check_container; then
        log "${RED}Container ${CONTAINER_NAME} not running!${NC}"
        return 1
    fi

    # Check SLAM process
    if ! check_slam_process; then
        log "${YELLOW}SLAM process not running${NC}"
        restart_slam
        return $?
    fi

    # Check map publishing
    if ! check_map_publishing; then
        log "${YELLOW}Map not publishing (SLAM may be stuck)${NC}"
        restart_slam
        return $?
    fi

    log "${GREEN}SLAM OK - map publishing${NC}"
    return 0
}

# Main
case "$1" in
    --daemon)
        log "Starting SLAM monitor daemon (checking every ${CHECK_INTERVAL}s)"
        log "Press Ctrl+C to stop"

        while true; do
            check_and_fix_slam
            sleep ${CHECK_INTERVAL}
        done
        ;;
    --check)
        # Single check, exit with status
        check_and_fix_slam
        exit $?
        ;;
    *)
        # Default: single check and fix
        check_and_fix_slam
        exit $?
        ;;
esac
