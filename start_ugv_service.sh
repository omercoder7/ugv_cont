#!/bin/bash
# start_ugv_service.sh - Starts the UGV container using docker run

CONTAINER_NAME="ugv_rpi_ros_humble"
IMAGE_NAME="ugv_beast_arm64:humble"

# --- X11 SETUP ---
# Detect and configure X11 forwarding for GUI applications
if [ -z "${DISPLAY}" ]; then
    echo "⚠️  WARNING: DISPLAY variable is not set. Setting to :0 (headless)."
    echo "   GUI applications (Rviz2, Gazebo) may not work without proper X11 forwarding."
    export DISPLAY=:0
else
    echo "✅ DISPLAY is set to: ${DISPLAY}"
    # Allow Docker containers to connect to X server
    xhost +local:docker 2>/dev/null || echo "⚠️  xhost not available, X11 forwarding may not work"
fi

# Check if container already exists
if docker ps -a --filter "name=^${CONTAINER_NAME}$" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    # Container exists, check if it's running
    if docker ps --filter "name=^${CONTAINER_NAME}$" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "✅ Container '${CONTAINER_NAME}' is already running."
        echo "   Use './enter_container.sh' to enter the container."
        exit 0
    else
        echo "🔄 Container '${CONTAINER_NAME}' exists but is stopped. Starting it..."
        docker start ${CONTAINER_NAME}
        if [ $? -eq 0 ]; then
            echo "✅ Container started successfully. Use './enter_container.sh' to enter."
            exit 0
        else
            echo "❌ Failed to start existing container. Check 'docker logs ${CONTAINER_NAME}' for details."
            exit 1
        fi
    fi
fi

# Container doesn't exist, create and start it
echo "🚀 Starting UGV container '${CONTAINER_NAME}' in detached mode..."

docker run -d \
  --name ${CONTAINER_NAME} \
  --privileged \
  --network host \
  --device /dev/ttyAMA0:/dev/ttyAMA0 \
  --device /dev/ttyACM0:/dev/ttyACM0 2>/dev/null \
  -e DISPLAY=${DISPLAY} \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=/root/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ${HOME}/.Xauthority:/root/.Xauthority 2>/dev/null \
  -v $(pwd)/ugv_data:/root/ugv_ws/ugv_data \
  --restart unless-stopped \
  --entrypoint /bin/bash \
  ${IMAGE_NAME} \
  -c "source /opt/ros/humble/setup.bash && source /root/ugv_ws/install/setup.bash && tail -f /dev/null"

if [ $? -eq 0 ]; then
    echo "✅ UGV container started successfully!"
    echo "   Use './enter_container.sh' to enter the running environment."
else
    echo "❌ Failed to start container. Check 'docker logs ${CONTAINER_NAME}' for details."
    exit 1
fi
