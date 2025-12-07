#!/bin/bash
# ensure_bringup.sh - Ensure robot bringup is running (no duplicates)
# This is a helper script called by auto_scan.py and other scripts
# to avoid duplicating bringup logic.
#
# Usage:
#   ./ensure_bringup.sh          # Start bringup if not running
#   ./ensure_bringup.sh --check  # Just check status, don't start
#   ./ensure_bringup.sh --wait   # Start and wait for topics
#
# Exit codes:
#   0 - Bringup running and topics available
#   1 - Failed to start or topics not available

CONTAINER_NAME="ugv_rpi_ros_humble"
MODE="${1:-start}"

# Check if container is running
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Start it with: ./start_ugv_service.sh"
    exit 1
fi

check_topics() {
    # Check if /scan topic exists
    SCAN=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null | grep -q '/scan' && echo 'yes'" 2>/dev/null)
    if [ "$SCAN" = "yes" ]; then
        return 0
    fi
    return 1
}

count_bringup_processes() {
    # Count how many bringup_lidar launch processes are running
    # Count only the python3 ros2 launch processes (not bash wrappers)
    local count
    count=$(docker exec ${CONTAINER_NAME} bash -c "ps aux | grep -E 'python3.*ros2 launch ugv_bringup bringup_lidar' | grep -v grep | wc -l" 2>/dev/null)
    count=$(echo "$count" | tr -d '[:space:]')
    if [ -z "$count" ] || ! [[ "$count" =~ ^[0-9]+$ ]]; then
        echo "0"
    else
        echo "$count"
    fi
}

check_bringup_process() {
    # Check if bringup nodes are running (ugv_bringup, ugv_driver, ldlidar, etc)
    # The ros2 launch process may exit after starting nodes, so check the nodes themselves
    if docker exec ${CONTAINER_NAME} pgrep -f "ugv_bringup" > /dev/null 2>&1; then
        return 0
    fi
    if docker exec ${CONTAINER_NAME} pgrep -f "ldlidar" > /dev/null 2>&1; then
        return 0
    fi
    return 1
}

kill_duplicates() {
    # Kill duplicate bringup processes, keeping only the oldest one
    local count=$(count_bringup_processes)
    if [ "$count" -gt 1 ]; then
        echo "Warning: Found $count duplicate bringup processes, cleaning up..."
        # Kill all bringup processes except the oldest
        docker exec ${CONTAINER_NAME} bash -c "
            pids=\$(pgrep -f 'bringup_lidar' | sort -n)
            first=true
            for pid in \$pids; do
                if \$first; then
                    first=false
                    continue
                fi
                kill \$pid 2>/dev/null
            done
        " 2>/dev/null
        sleep 2

        # If still duplicates, kill all and restart fresh
        count=$(count_bringup_processes)
        if [ "$count" -gt 1 ]; then
            echo "Still have duplicates, killing all..."
            docker exec ${CONTAINER_NAME} pkill -f "bringup_lidar" 2>/dev/null
            docker exec ${CONTAINER_NAME} pkill -f "joint_state_publisher" 2>/dev/null
            docker exec ${CONTAINER_NAME} pkill -f "ugv_driver" 2>/dev/null
            docker exec ${CONTAINER_NAME} pkill -f "rf2o_laser_odometry" 2>/dev/null
            sleep 2
            return 1  # Signal that we need to start fresh
        fi
    fi

    # Also check for duplicate rf2o nodes using ros2 node list
    local rf2o_count
    rf2o_count=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null | grep -c rf2o_laser_odometry" 2>/dev/null || echo "0")
    rf2o_count=$(echo "$rf2o_count" | tr -d '[:space:]')
    if [ "$rf2o_count" -gt 1 ]; then
        echo "Warning: Found $rf2o_count duplicate rf2o nodes, killing all rf2o..."
        docker exec ${CONTAINER_NAME} pkill -f "rf2o" 2>/dev/null
        sleep 2
    fi

    return 0
}

start_bringup() {
    # First check for and kill any duplicates
    kill_duplicates

    # Check if already running properly
    if check_bringup_process && check_topics; then
        echo "Bringup already running properly"
        return 0
    fi

    echo "Starting robot bringup..."
    docker exec -d ${CONTAINER_NAME} /bin/bash -c "
        source /opt/ros/humble/setup.bash && \
        source /root/ugv_ws/install/setup.bash && \
        export UGV_MODEL=ugv_beast && \
        export LDLIDAR_MODEL=ld19 && \
        ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true > /tmp/bringup.log 2>&1
    "
}

wait_for_topics() {
    local max_wait=15
    local waited=0

    while [ $waited -lt $max_wait ]; do
        if check_topics; then
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
        echo -ne "\rWaiting for topics... ${waited}s"
    done
    echo ""
    return 1
}

case "${MODE}" in
    --check)
        # Just check status (also clean duplicates)
        kill_duplicates
        if check_bringup_process && check_topics; then
            echo "Bringup: RUNNING"
            exit 0
        else
            echo "Bringup: NOT RUNNING"
            exit 1
        fi
        ;;
    --wait|start|"")
        # First clean up any duplicates
        kill_duplicates

        # Check if already running properly with topics
        if check_bringup_process && check_topics; then
            echo "Bringup already running"
            exit 0
        fi

        # Start if not running
        if ! check_bringup_process; then
            start_bringup
            sleep 3
        fi

        if wait_for_topics; then
            echo "Bringup ready"
            exit 0
        else
            echo "Error: Topics not available after waiting"
            echo "Check /tmp/bringup.log inside container for errors"
            exit 1
        fi
        ;;
    *)
        echo "Usage: $0 [--check|--wait]"
        exit 1
        ;;
esac
