#!/bin/bash
set -e

# Source the main ROS distribution environment
source "/opt/ros/humble/setup.bash"

# Source the compiled workspace if it exists.
# The user is 'ros' in the final Docker image.
if [ -f "/home/ros/ugv_ws/install/setup.bash" ]; then
  source "/home/ros/ugv_ws/install/setup.bash"
fi

# Execute the main command passed via docker-compose (e.g., /bin/bash)
exec "$@"
