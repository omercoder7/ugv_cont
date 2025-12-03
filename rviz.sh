#!/bin/bash
# rviz.sh - Launch RViz with different visualization modes
# Usage: ./rviz.sh [mode]
#
# Modes:
#   lidar      - View LiDAR scan data
#   bringup    - Basic robot bringup view (lidar + odom + tf)
#   ekf        - Start EKF sensor fusion (LiDAR + IMU) without SLAM
#   slam       - SLAM mapping view (2D) with EKF fusion
#   slam3d     - SLAM mapping view (3D)
#   nav        - Navigation view (2D)
#   nav3d      - Navigation view (3D)
#   camera     - Camera image view
#   imu        - IMU data view
#   full       - Full sensor view (lidar + camera + imu)
#   custom     - Launch with custom .rviz file (provide path as 2nd arg)
#
# Examples:
#   ./rviz.sh lidar
#   ./rviz.sh slam
#   ./rviz.sh custom /path/to/config.rviz

CONTAINER_NAME="ugv_rpi_ros_humble"
MODE="${1:-bringup}"

# Check if container is running
if ! docker ps --filter "name=${CONTAINER_NAME}" --filter "status=running" --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Please start it using './start_ugv_service.sh' first."
    exit 1
fi

# X11 setup - auto-detect SSH client IP for DISPLAY
if [ -z "${DISPLAY}" ]; then
    # Try to auto-detect SSH client IP from SSH_CONNECTION
    if [ -n "${SSH_CONNECTION}" ]; then
        SSH_CLIENT_IP=$(echo "${SSH_CONNECTION}" | awk '{print $1}')
        export DISPLAY="${SSH_CLIENT_IP}:0.0"
        echo "Auto-detected SSH client IP: ${SSH_CLIENT_IP}"
    else
        # Try to get from 'who' command
        SSH_CLIENT_IP=$(who | grep -m1 "$(whoami)" | grep -oP '\(\K[0-9.]+(?=\))' 2>/dev/null)
        if [ -n "${SSH_CLIENT_IP}" ]; then
            export DISPLAY="${SSH_CLIENT_IP}:0.0"
            echo "Auto-detected client IP from 'who': ${SSH_CLIENT_IP}"
        else
            echo "Error: DISPLAY not set and could not auto-detect SSH client IP."
            echo "Please set DISPLAY manually: export DISPLAY=<your_ip>:0.0"
            exit 1
        fi
    fi
fi

echo "Using DISPLAY=${DISPLAY}"

# Copy xauth cookie to container for X11 authentication
XAUTH_COOKIE=$(xauth list "${DISPLAY}" 2>/dev/null | head -1 | awk '{print $3}')
if [ -n "${XAUTH_COOKIE}" ]; then
    docker exec ${CONTAINER_NAME} bash -c "
        touch /root/.Xauthority && \
        xauth add ${DISPLAY} MIT-MAGIC-COOKIE-1 ${XAUTH_COOKIE}
    " 2>/dev/null
fi

xhost +local:docker 2>/dev/null
xhost + 2>/dev/null

# RViz config paths
RVIZ_LIDAR="/root/ugv_ws/install/ldlidar/share/ldlidar/rviz/view.rviz"
RVIZ_BRINGUP="/root/ugv_ws/install/ugv_bringup/share/ugv_bringup/rviz/view_bringup.rviz"
RVIZ_SLAM_2D="/root/ugv_ws/install/ugv_slam/share/ugv_slam/rviz/view_slam_2d.rviz"
RVIZ_SLAM_3D="/root/ugv_ws/install/ugv_slam/share/ugv_slam/rviz/view_slam_3d.rviz"
RVIZ_NAV_2D="/root/ugv_ws/install/ugv_nav/share/ugv_nav/rviz/view_nav_2d.rviz"
RVIZ_NAV_3D="/root/ugv_ws/install/ugv_nav/share/ugv_nav/rviz/view_nav_3d.rviz"
RVIZ_DESC="/root/ugv_ws/install/ugv_description/share/ugv_description/rviz/view_description.rviz"

show_help() {
    echo "UGV RViz Launcher"
    echo ""
    echo "Usage: ./rviz.sh [mode] [options]"
    echo ""
    echo "Modes:"
    echo "  lidar       - View LiDAR scan data"
    echo "  bringup     - Basic robot bringup view (default)"
    echo "  ekf         - Start EKF sensor fusion (LiDAR + IMU) without SLAM"
    echo "  slam        - Full SLAM+Nav2 using manufacturer's launch"
    echo "  slam-opt    - OPTIMIZED SLAM with better parameters (RECOMMENDED)"
    echo "                Use 'slam-opt new_map' to start with a fresh map"
    echo "  slam-simple - Simple SLAM without Nav2 (requires bringup running)"
    echo "  slam3d      - SLAM+Nav2 with 3D visualization"
    echo "  nav         - Navigation view (2D)"
    echo "  nav3d       - Navigation view (3D)"
    echo "  camera      - Camera image view"
    echo "  imu         - IMU data visualization"
    echo "  full        - Full sensor view"
    echo "  robot       - Robot model only"
    echo "  custom      - Custom .rviz file (provide path as 2nd arg)"
    echo ""
    echo "Examples:"
    echo "  ./rviz.sh lidar            # Just view LiDAR"
    echo "  ./rviz.sh slam             # Full SLAM with proper odometry"
    echo "  ./rviz.sh slam-opt         # Optimized SLAM (continues existing map)"
    echo "  ./rviz.sh slam-opt new_map # Optimized SLAM with fresh map"
    echo "  ./rviz.sh slam-simple      # Simple SLAM (if bringup already running)"
    echo "  ./rviz.sh custom /path/to/my_config.rviz"
}

launch_rviz() {
    local config="$1"
    local extra_cmd="$2"

    echo "Launching RViz in ${MODE} mode..."
    echo "Using DISPLAY=${DISPLAY}"

    # Use software rendering for X11 forwarding (required for RPi)
    docker exec -it \
        -e DISPLAY=${DISPLAY} \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        -e QT_X11_NO_MITSHM=1 \
        ${CONTAINER_NAME} /bin/bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ugv_ws/install/setup.bash && \
    export UGV_MODEL=ugv_beast && \
    export LDLIDAR_MODEL=ld19 && \
    ${extra_cmd}
    rviz2 -d ${config}
    "
}

launch_rviz_no_config() {
    local extra_cmd="$1"

    echo "Launching RViz in ${MODE} mode..."
    echo "Using DISPLAY=${DISPLAY}"

    docker exec -it -e DISPLAY=${DISPLAY} ${CONTAINER_NAME} /bin/bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ugv_ws/install/setup.bash && \
    export UGV_MODEL=ugv_beast && \
    export LDLIDAR_MODEL=ld19 && \
    ${extra_cmd}
    rviz2
    "
}

case "${MODE}" in
    lidar)
        launch_rviz "${RVIZ_LIDAR}"
        ;;
    bringup)
        launch_rviz "${RVIZ_BRINGUP}"
        ;;
    ekf)
        echo "Starting EKF sensor fusion (LiDAR + IMU)..."

        # Copy custom EKF config into container
        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        if [ -f "${SCRIPT_DIR}/ekf_lidar_imu.yaml" ]; then
            echo "Copying EKF config into container..."
            docker cp "${SCRIPT_DIR}/ekf_lidar_imu.yaml" ${CONTAINER_NAME}:/tmp/ekf_lidar_imu.yaml
        else
            echo "Error: ekf_lidar_imu.yaml not found in ${SCRIPT_DIR}"
            exit 1
        fi

        # Start EKF for IMU fusion (fuses /odom_rf2o + /imu/data -> /odom)
        echo "Starting EKF node..."
        echo "  Input:  /odom_rf2o (laser odometry) + /imu/data (IMU)"
        echo "  Output: /odom (fused odometry)"
        docker exec -d ${CONTAINER_NAME} /bin/bash -c "
        source /opt/ros/humble/setup.bash && \
        source /root/ugv_ws/install/setup.bash && \
        ros2 run robot_localization ekf_node --ros-args \
          --params-file /tmp/ekf_lidar_imu.yaml \
          -r /odometry/filtered:=/odom
        " 2>/dev/null
        sleep 2

        echo "EKF node started. Launching RViz..."
        launch_rviz "${RVIZ_BRINGUP}"
        ;;
    slam)
        echo "Starting SLAM using manufacturer's Nav2 launch..."
        echo ""
        echo "This uses the proper launch chain:"
        echo "  - bringup_lidar.launch.py (LiDAR + base_node + rf2o)"
        echo "  - Nav2 SLAM (slam_toolbox via nav2_bringup)"
        echo "  - Odometry from wheel encoders (/odom)"
        echo ""
        echo "Using DISPLAY=${DISPLAY}"

        # Use manufacturer's slam_nav.launch.py for proper SLAM
        docker exec -it \
            -e DISPLAY=${DISPLAY} \
            -e LIBGL_ALWAYS_SOFTWARE=1 \
            -e QT_X11_NO_MITSHM=1 \
            ${CONTAINER_NAME} /bin/bash -c "
        source /opt/ros/humble/setup.bash && \
        source /root/ugv_ws/install/setup.bash && \
        export UGV_MODEL=ugv_beast && \
        export LDLIDAR_MODEL=ld19 && \
        ros2 launch ugv_nav slam_nav.launch.py \
          use_rviz:=true \
          rviz_config:=nav_2d \
          slam:=True \
          use_sim_time:=false
        "
        ;;
    slam-simple)
        echo "Starting simple SLAM (slam_toolbox only, no Nav2)..."
        echo "Note: This assumes bringup_lidar is already running."
        echo ""

        # Just start slam_toolbox
        docker exec -d ${CONTAINER_NAME} /bin/bash -c "
        source /opt/ros/humble/setup.bash && \
        source /root/ugv_ws/install/setup.bash && \
        ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
        " 2>/dev/null
        sleep 5
        launch_rviz "${RVIZ_SLAM_2D}"
        ;;
    slam-opt|slam-optimized)
        NEW_MAP="$2"

        echo "Starting OPTIMIZED SLAM with improved parameters..."
        echo ""
        echo "Using DISPLAY=${DISPLAY}"

        # Copy configs to container
        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        docker cp "${SCRIPT_DIR}/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml
        docker cp "${SCRIPT_DIR}/view_slam_2d.rviz" ${CONTAINER_NAME}:/tmp/view_slam_2d.rviz 2>/dev/null

        # Handle new_map option - kill existing SLAM and reset odometry
        if [ "${NEW_MAP}" = "new_map" ] || [ "${NEW_MAP}" = "new" ] || [ "${NEW_MAP}" = "--new-map" ]; then
            echo ""
            echo "*** NEW MAP MODE - Resetting SLAM and odometry ***"
            echo ""

            # Kill ALL ROS2 related processes to get a clean slate
            # Using killall -9 for more reliable cleanup
            echo "Stopping all ROS2 nodes..."
            docker exec ${CONTAINER_NAME} bash -c "killall -9 python3 2>/dev/null; killall -9 rviz2 2>/dev/null" 2>/dev/null
            sleep 2

            # Double-check with pkill for any remaining processes
            docker exec ${CONTAINER_NAME} pkill -9 -f "slam_toolbox" 2>/dev/null
            docker exec ${CONTAINER_NAME} pkill -9 -f "rf2o" 2>/dev/null
            docker exec ${CONTAINER_NAME} pkill -9 -f "ldlidar" 2>/dev/null
            docker exec ${CONTAINER_NAME} pkill -9 -f "base_node" 2>/dev/null
            docker exec ${CONTAINER_NAME} pkill -9 -f "robot_state" 2>/dev/null
            docker exec ${CONTAINER_NAME} pkill -9 -f "joint_state" 2>/dev/null
            docker exec ${CONTAINER_NAME} pkill -9 -f "ugv_driver" 2>/dev/null
            sleep 2

            echo "All nodes stopped."

            # Start fresh bringup
            echo "Starting robot bringup..."
            docker exec -d ${CONTAINER_NAME} /bin/bash -c "
            source /opt/ros/humble/setup.bash && \
            source /root/ugv_ws/install/setup.bash && \
            export UGV_MODEL=ugv_beast && \
            export LDLIDAR_MODEL=ld19 && \
            ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true
            " 2>/dev/null
            sleep 5

            # Start fresh SLAM
            echo "Starting fresh SLAM with optimized config..."
            docker exec -d ${CONTAINER_NAME} /bin/bash -c "
            source /opt/ros/humble/setup.bash && \
            source /root/ugv_ws/install/setup.bash && \
            ros2 launch slam_toolbox online_async_launch.py \
              use_sim_time:=false \
              slam_params_file:=/tmp/slam_toolbox_optimized.yaml
            " 2>/dev/null
            sleep 3

            echo "Fresh map started!"
        else
            # Normal mode - check if already running
            # Check if bringup is already running
            if ! docker exec ${CONTAINER_NAME} pgrep -f "bringup_lidar" > /dev/null 2>&1; then
                echo "Starting robot bringup..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                source /root/ugv_ws/install/setup.bash && \
                export UGV_MODEL=ugv_beast && \
                export LDLIDAR_MODEL=ld19 && \
                ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true
                " 2>/dev/null
                sleep 5
            else
                echo "Bringup already running, skipping..."
            fi

            # Check if SLAM is already running
            if ! docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
                echo "Starting SLAM with optimized config..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                source /root/ugv_ws/install/setup.bash && \
                ros2 launch slam_toolbox online_async_launch.py \
                  use_sim_time:=false \
                  slam_params_file:=/tmp/slam_toolbox_optimized.yaml
                " 2>/dev/null
                sleep 3
            else
                echo "SLAM already running, skipping..."
            fi
        fi

        echo "Launching RViz with manufacturer's SLAM config..."
        launch_rviz "/tmp/view_slam_2d.rviz"
        ;;
    slam3d)
        echo "Starting SLAM 3D using manufacturer's Nav2 launch..."
        echo "Using DISPLAY=${DISPLAY}"
        docker exec -it \
            -e DISPLAY=${DISPLAY} \
            -e LIBGL_ALWAYS_SOFTWARE=1 \
            -e QT_X11_NO_MITSHM=1 \
            ${CONTAINER_NAME} /bin/bash -c "
        source /opt/ros/humble/setup.bash && \
        source /root/ugv_ws/install/setup.bash && \
        export UGV_MODEL=ugv_beast && \
        export LDLIDAR_MODEL=ld19 && \
        ros2 launch ugv_nav slam_nav.launch.py \
          use_rviz:=true \
          rviz_config:=nav_3d \
          slam:=True \
          use_sim_time:=false
        "
        ;;
    nav)
        launch_rviz "${RVIZ_NAV_2D}"
        ;;
    nav3d)
        launch_rviz "${RVIZ_NAV_3D}"
        ;;
    camera)
        echo "Make sure camera is running (./rviz.sh will show image topic)"
        launch_rviz_no_config ""
        ;;
    imu)
        launch_rviz_no_config ""
        ;;
    full)
        launch_rviz "${RVIZ_BRINGUP}"
        ;;
    robot)
        launch_rviz "${RVIZ_DESC}"
        ;;
    custom)
        if [ -z "$2" ]; then
            echo "Error: Please provide path to .rviz config file"
            echo "Usage: ./rviz.sh custom /path/to/config.rviz"
            exit 1
        fi
        launch_rviz "$2"
        ;;
    help|-h|--help)
        show_help
        exit 0
        ;;
    *)
        echo "Unknown mode: ${MODE}"
        show_help
        exit 1
        ;;
esac
