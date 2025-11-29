#!/bin/bash
# enter_container.sh - Attaches a new terminal to the running UGV container.

CONTAINER_NAME="ugv_rpi_ros_humble"

# 1. Check if the container is currently running
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "🛑 Error: Container '${CONTAINER_NAME}' is not currently running."
    echo "Please start it using './start_ugv_service.sh' first."
    exit 1
fi

echo "🔗 Connecting to running container '${CONTAINER_NAME}'..."

# 2. Execute bash inside the container with ROS2 environment loaded and UGV environment variables set
docker exec -it ${CONTAINER_NAME} /bin/bash -c "
source /opt/ros/humble/setup.bash && \
source /root/ugv_ws/install/setup.bash && \
export UGV_WS_DIR=/home/ws/ugv_ws && \
export UGV_MODEL=ugv_beast && \
export LDLIDAR_MODEL=ld19 && \
exec /bin/bash
"

echo "✅ Exited container."
