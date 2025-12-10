#!/bin/bash
# keyboard_control.sh - Control the UGV robot with keyboard
# Runs both ugv_driver (for serial communication) and keyboard_ctrl inside container

CONTAINER_NAME="ugv_rpi_ros_humble"

# Check if container is running
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Please start it using './start_ugv_service.sh' first."
    exit 1
fi

# Cleanup function
cleanup() {
    echo ""
    echo "Stopping robot..."
    # Send stop directly to serial as backup
    echo '{"T":"13","X":0,"Z":0}' > /dev/ttyAMA0 2>/dev/null
    # Kill processes in container
    docker exec ${CONTAINER_NAME} pkill -f ugv_driver 2>/dev/null
    docker exec ${CONTAINER_NAME} pkill -f keyboard_ctrl 2>/dev/null
    echo "Done."
}

trap cleanup EXIT

# Kill any existing processes
docker exec ${CONTAINER_NAME} pkill -f ugv_driver 2>/dev/null
docker exec ${CONTAINER_NAME} pkill -f keyboard_ctrl 2>/dev/null
sleep 1

# Start the UGV driver in background (for serial communication)
echo "Starting UGV driver..."
docker exec -d ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
ros2 run ugv_bringup ugv_driver
"
sleep 2

# Check if driver is running
if ! docker exec ${CONTAINER_NAME} pgrep -f ugv_driver > /dev/null; then
    echo "Warning: ugv_driver may not have started properly."
fi

echo "Starting keyboard control..."
echo "Use: u/i/o, j/k/l, m/,/. to move. Space=stop, Ctrl+C=quit"
echo ""

# Run keyboard_ctrl interactively with proper TTY
docker exec -it ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
ros2 run ugv_tools keyboard_ctrl
"
