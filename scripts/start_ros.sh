#!/bin/bash
# start_ros.sh - Start container and ROS2 nodes with EKF sensor fusion
#
# IMPORTANT: This script restarts the Docker container to clear zombie processes.
# Killing processes alone does NOT remove zombies - only container restart does.
#
# Usage:
#   ./start_ros.sh           # Start with EKF sensor fusion
#   ./start_ros.sh new_map   # Restart container and start fresh

CONTAINER_NAME="ugv_rpi_ros_humble"
SLAM_MAX_RETRIES=3
NEW_MAP=false

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
        new_map|new|--new-map)
            NEW_MAP=true
            shift
            ;;
    esac
done

echo "=== ROS2 Start Script (EKF Mode) ==="
echo "Mode: EKF (IMU + Laser Odometry fusion)"
if [ "$NEW_MAP" = true ]; then
    echo "Option: NEW MAP (full restart)"
fi
echo ""

# Check container
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    log_err "Container '${CONTAINER_NAME}' is not running."
    exit 1
fi

# Show current state (with timeout to prevent hang)
log "Current ROS2 nodes (before restart):"
timeout 5 docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" 2>/dev/null | sort | uniq -c || echo "  (timeout getting nodes)"
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

# Clear FastDDS shared memory to prevent stale node registrations
log "Clearing FastDDS shared memory cache..."
docker exec ${CONTAINER_NAME} bash -c "rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null" || true
rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

# Reset ROS2 daemon inside container to clear DDS discovery cache
log "Resetting ROS2 daemon (clears DDS cache)..."
timeout 5 docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 daemon stop && ros2 daemon start" 2>/dev/null || true
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
# START BRINGUP (EKF MODE)
# ============================================================================
echo ""
log "=== Starting Robot Bringup (EKF) ==="

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
BRINGUP_WAIT=10

# Wait for bringup with progress
log "Waiting for bringup to initialize (${BRINGUP_WAIT}s)..."
for i in $(seq 1 ${BRINGUP_WAIT}); do
    echo -ne "\r  Progress: ${i}/${BRINGUP_WAIT}s"
    sleep 1
done
echo ""

# Verify bringup is running
if docker exec ${CONTAINER_NAME} pgrep -f "ros2" > /dev/null 2>&1; then
    log_ok "Bringup process is running"
else
    log_err "Bringup FAILED - check /tmp/bringup.log"
    docker exec ${CONTAINER_NAME} tail -10 /tmp/bringup.log 2>/dev/null || true
    exit 1
fi

# ============================================================================
# VERIFY TF TREE IS READY (Critical for SLAM)
# ============================================================================
echo ""
log "=== Verifying TF Tree ==="

TF_READY=false
for i in $(seq 1 8); do
    TF_CHECK=$(timeout 2 docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 1 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1" 2>/dev/null || echo "")
    if echo "${TF_CHECK}" | grep -q "Translation"; then
        TF_READY=true
        break
    fi
    echo -ne "\r  Waiting for TF: ${i}/8s"
    sleep 1
done
echo ""

if [ "${TF_READY}" = true ]; then
    log_ok "TF tree is ready (odom->base_footprint)"
else
    log_warn "TF tree not fully ready - continuing anyway"
fi

# Verify /scan topic is publishing
SCAN_CHECK=$(timeout 3 docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null | grep -q '/scan' && echo 'found'" 2>/dev/null || echo "")
if [ "${SCAN_CHECK}" = "found" ]; then
    log_ok "/scan topic exists"
else
    log_warn "/scan topic not found - check LiDAR"
fi

# ============================================================================
# START SLAM WITH RETRY MECHANISM
# ============================================================================
echo ""
log "=== Starting SLAM Toolbox ==="

start_slam() {
    docker exec ${CONTAINER_NAME} pkill -9 -f "slam_toolbox" 2>/dev/null || true
    docker exec ${CONTAINER_NAME} pkill -9 -f "async_slam" 2>/dev/null || true
    sleep 1

    docker exec -d ${CONTAINER_NAME} /bin/bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ugv_ws/install/setup.bash && \
    ros2 launch slam_toolbox online_async_launch.py \
      use_sim_time:=false \
      slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1
    "
}

verify_slam_process() {
    local live_count
    live_count=$(docker exec ${CONTAINER_NAME} bash -c "ps aux | grep -E 'async_slam_toolbox_node|slam_toolbox' | grep -v '<defunct>' | grep -v grep | wc -l" 2>/dev/null || echo "0")
    [ "${live_count}" -gt 0 ]
}

verify_map_topic() {
    timeout 3 docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null | grep -q '^/map$'" 2>/dev/null
}

verify_map_publishing() {
    local result
    result=$(timeout 8 docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /map --once 2>/dev/null | grep -c 'frame_id: map'" 2>/dev/null || echo "0")
    [ "${result}" -gt 0 ]
}

# Try to start SLAM with retries
SLAM_SUCCESS=false
for attempt in $(seq 1 ${SLAM_MAX_RETRIES}); do
    log "SLAM startup attempt ${attempt}/${SLAM_MAX_RETRIES}..."

    start_slam

    log "  Waiting for SLAM to initialize..."
    sleep 6

    if ! verify_slam_process; then
        log_warn "  SLAM process not running"
        docker exec ${CONTAINER_NAME} tail -5 /tmp/slam.log 2>/dev/null | grep -i "error\|exception\|failed" | head -3 || true
        continue
    fi
    log "  SLAM process started"

    sleep 2
    if ! verify_map_topic; then
        log_warn "  /map topic not created yet, waiting..."
        sleep 3
    fi

    log "  Verifying /map is publishing..."
    if verify_map_publishing; then
        log_ok "SLAM is running and publishing map!"
        SLAM_SUCCESS=true
        break
    else
        log_warn "  /map not publishing data"
        SLAM_ERRORS=$(docker exec ${CONTAINER_NAME} tail -20 /tmp/slam.log 2>/dev/null | grep -i "error\|exception\|failed\|warn" | head -3 || echo "")
        if [ -n "${SLAM_ERRORS}" ]; then
            echo "  Log issues: ${SLAM_ERRORS}"
        fi
    fi

    if [ ${attempt} -lt ${SLAM_MAX_RETRIES} ]; then
        log "  Retrying in 2 seconds..."
        sleep 2
    fi
done

if [ "${SLAM_SUCCESS}" = false ]; then
    log_err "SLAM failed to start after ${SLAM_MAX_RETRIES} attempts!"
    log "Attempting aggressive manual restart..."

    docker exec ${CONTAINER_NAME} pkill -9 -f "slam" 2>/dev/null || true
    sleep 3

    log "Starting SLAM directly..."
    docker exec -d ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash && \
        source /root/ugv_ws/install/setup.bash && \
        ros2 launch slam_toolbox online_async_launch.py \
          use_sim_time:=false \
          slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1
    "

    log "Waiting for manual SLAM start (10s)..."
    for i in $(seq 1 10); do
        echo -ne "\r  Progress: ${i}/10s"
        sleep 1
    done
    echo ""

    if verify_slam_process; then
        log "SLAM process running, checking /map..."
        sleep 2
        if verify_map_publishing; then
            log_ok "SLAM started on manual attempt!"
            SLAM_SUCCESS=true
        else
            if verify_map_topic; then
                log_warn "SLAM running, /map topic exists but not publishing yet"
                log "Give it 10-15 more seconds, then check with: ./ensure_slam.sh --check"
                SLAM_SUCCESS=true
            else
                log_err "SLAM process running but no /map topic"
                docker exec ${CONTAINER_NAME} tail -10 /tmp/slam.log 2>/dev/null || true
            fi
        fi
    else
        log_err "SLAM truly failed to start"
        log "Check logs: docker exec ${CONTAINER_NAME} tail -30 /tmp/slam.log"
        docker exec ${CONTAINER_NAME} tail -10 /tmp/slam.log 2>/dev/null || true
    fi
fi

# ============================================================================
# VERIFY EKF
# ============================================================================
echo ""
log "=== Verifying EKF ==="

if docker exec ${CONTAINER_NAME} pgrep -f "ekf" > /dev/null 2>&1; then
    log_ok "EKF is running"
else
    log_warn "EKF not detected - may be part of bringup launch"
fi

ODOM_CHECK=$(timeout 3 docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null | grep -q '^/odom$' && echo 'found'" 2>/dev/null || echo "")
if [ "${ODOM_CHECK}" = "found" ]; then
    log_ok "/odom topic exists"
else
    log_warn "/odom topic not found"
fi

# ============================================================================
# FINAL STATUS
# ============================================================================
echo ""
log "=== Final Status ==="

DUPLICATE_NODES=$(timeout 5 docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" 2>/dev/null | sort | uniq -d)
if [ -n "${DUPLICATE_NODES}" ]; then
    log_warn "Duplicate node entries detected (stale DDS cache):"
    echo "${DUPLICATE_NODES}" | while read node; do echo "  - ${node}"; done
    log "These are harmless phantom entries that will clear in ~30 seconds"
fi

TOPICS=$(timeout 3 docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null" 2>/dev/null || echo "")

if docker exec ${CONTAINER_NAME} pgrep -f "ros2" > /dev/null 2>&1; then
    log_ok "Bringup: RUNNING"
else
    log_err "Bringup: NOT RUNNING"
fi

if echo "${TOPICS}" | grep -q "^/scan$"; then
    log_ok "/scan: AVAILABLE"
else
    log_err "/scan: NOT AVAILABLE"
fi

if echo "${TOPICS}" | grep -q "^/odom$"; then
    log_ok "/odom: AVAILABLE"
else
    log_err "/odom: NOT AVAILABLE"
fi

if verify_slam_process; then
    log_ok "SLAM: RUNNING"
else
    log_err "SLAM: NOT RUNNING"
fi

if echo "${TOPICS}" | grep -q "^/map$"; then
    log_ok "/map: AVAILABLE"
else
    log_err "/map: NOT AVAILABLE"
fi

echo ""
if [ "${SLAM_SUCCESS}" = true ]; then
    log_ok "=== ALL SYSTEMS GO! ==="
else
    log_warn "=== Started with warnings - SLAM may need manual check ==="
fi

echo ""
log "=== Done (took ~45 seconds) ==="
