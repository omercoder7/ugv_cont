#!/bin/bash
# enter_container.sh - Attaches a new terminal to the running UGV service.

SERVICE_NAME="ugv_beast"
COMPOSE_FILE="docker-compose.yaml"

# 1. Check if the service is currently running
if ! docker compose -f $COMPOSE_FILE ps --filter "status=running" --format '{{.Name}}' | grep -q "${SERVICE_NAME}"; then
    echo "🛑 Error: Service '$SERVICE_NAME' is not currently running."
    echo "Please start it using './start_ugv_services.sh' first."
    exit 1
fi

echo "Connecting to running service '$SERVICE_NAME' as user 'ros'..."

# 2. Execute bash inside the container service.
# -u ros: Executes the command as the non-root user 'ros'.
# The /bin/bash command is run, which automatically executes /usr/local/bin/entrypoint.sh 
# and sources the ROS environment setup.
docker compose -f $COMPOSE_FILE exec -u ros $SERVICE_NAME /bin/bash

echo "Exited container."
