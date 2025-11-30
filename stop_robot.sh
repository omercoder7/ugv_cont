#!/bin/bash
# stop_robot.sh - Emergency stop for UGV robot
# Sends stop command directly to serial port (bypasses ROS2)

SERIAL_PORT="/dev/ttyAMA0"

echo "Sending emergency stop..."

# Send stop command directly to serial port (JSON format the robot expects)
# T:13 is velocity command, X:0 Z:0 means stop
echo '{"T":"13","X":0,"Z":0}' > ${SERIAL_PORT}

# Also try via ROS2 if ugv_driver is running
docker exec ugv_rpi_ros_humble bash -c "source /opt/ros/humble/setup.bash && timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'" 2>/dev/null &

echo "Stop command sent!"
