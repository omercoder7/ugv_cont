#!/bin/bash
# start_ugv_services.sh - Starts the UGV service defined in docker-compose.yaml

COMPOSE_FILE="docker-compose.yaml"

# --- CRITICAL X11 SETUP ---
# Ensure X authority is available and DISPLAY is correctly set for GUI forwarding.
if [ -z "${DISPLAY}" ]; then
    echo "🛑 ERROR: DISPLAY variable is not set. GUI applications (Rviz2, Gazebo) will fail."
    echo "Ensure you are connected via SSH with X11 forwarding enabled (-X or -Y) or running locally."
    exit 1
fi

if [ ! -f "${HOME}/.Xauthority" ]; then
    echo "⚠️ WARNING: .Xauthority file not found. Creating a temporary xauth key."
    # Note: Creating a temporary key here is a simplified approach, but may require 
    # manual xhost configuration on some host systems.
    xauth generate :0 . trusted
fi


echo "🚀 Starting UGV Compose Service in detached mode..."
# 'up -d' starts the service in the background. --build ensures the latest Dockerfile version is used.
docker compose -f $COMPOSE_FILE up -d --build

if [ $? -eq 0 ]; then
    echo "✅ UGV service started. Use './enter_container.sh' to enter the running environment."
else
    echo "❌ Service failed to start. Check 'docker compose logs ugv_beast' for details."
fi
