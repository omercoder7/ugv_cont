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
# Extract display number from DISPLAY (e.g., localhost:10.0 -> 10)
DISPLAY_NUM=$(echo "${DISPLAY}" | sed 's/.*:\([0-9]*\).*/\1/')

# Try multiple xauth lookup methods
XAUTH_COOKIE=$(xauth list 2>/dev/null | grep ":${DISPLAY_NUM}" | head -1 | awk '{print $3}')
if [ -z "${XAUTH_COOKIE}" ]; then
    XAUTH_COOKIE=$(xauth list "${DISPLAY}" 2>/dev/null | head -1 | awk '{print $3}')
fi

if [ -n "${XAUTH_COOKIE}" ]; then
    echo "Found X11 auth cookie for display :${DISPLAY_NUM}"
    docker exec ${CONTAINER_NAME} bash -c "
        touch /root/.Xauthority && \
        xauth add ${DISPLAY} MIT-MAGIC-COOKIE-1 ${XAUTH_COOKIE} 2>/dev/null
        xauth add localhost:${DISPLAY_NUM} MIT-MAGIC-COOKIE-1 ${XAUTH_COOKIE} 2>/dev/null
        xauth add localhost:${DISPLAY_NUM}.0 MIT-MAGIC-COOKIE-1 ${XAUTH_COOKIE} 2>/dev/null
    " 2>/dev/null
else
    echo "Warning: Could not find X11 auth cookie"
fi

xhost +local:docker 2>/dev/null
xhost + 2>/dev/null

# Function to kill duplicate/zombie ROS2 nodes
cleanup_duplicate_nodes() {
    echo "Checking for duplicate ROS2 nodes..."

    # Get list of nodes
    NODES=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | sort)

    # Check for duplicates
    DUPS=$(echo "$NODES" | uniq -d)
    if [ -n "$DUPS" ]; then
        echo "Found duplicate nodes: $DUPS"

        # Kill specific duplicate processes
        if echo "$DUPS" | grep -q "rf2o"; then
            echo "  Killing duplicate rf2o_laser_odometry..."
            docker exec ${CONTAINER_NAME} pkill -f rf2o 2>/dev/null
            sleep 1
        fi
        if echo "$DUPS" | grep -q "cartographer"; then
            echo "  Killing duplicate cartographer nodes..."
            docker exec ${CONTAINER_NAME} pkill -f cartographer 2>/dev/null
            sleep 1
        fi
        if echo "$DUPS" | grep -q "slam_toolbox"; then
            echo "  Killing duplicate slam_toolbox..."
            docker exec ${CONTAINER_NAME} pkill -f slam_toolbox 2>/dev/null
            sleep 1
        fi
    fi

    # Check for zombie processes
    ZOMBIES=$(docker exec ${CONTAINER_NAME} ps aux 2>/dev/null | grep -c "<defunct>" || echo "0")
    if [ "${ZOMBIES}" -gt 0 ]; then
        echo "Warning: ${ZOMBIES} zombie processes found."
        echo "  Consider running with 'new_map' option to restart container."
    fi
}

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
    echo "  slam-opt    - OPTIMIZED SLAM with better parameters"
    echo "                Use 'slam-opt new_map' to start with a fresh map"
    echo "                Use 'slam-opt new_map --ekf' for EKF sensor fusion"
    echo "  slam-ekf    - SLAM with EKF triple fusion"
    echo "                Uses wheel encoders + LiDAR + IMU"
    echo "  slam-carto  - Google Cartographer SLAM (BEST ACCURACY)"
    echo "                More accurate than slam_toolbox, less reliant on odometry"
    echo "                Use 'slam-carto new_map' to start fresh"
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
    echo "  ./rviz.sh lidar             # Just view LiDAR"
    echo "  ./rviz.sh slam-carto new_map # BEST: Cartographer SLAM (fresh map)"
    echo "  ./rviz.sh slam-opt new_map  # Optimized slam_toolbox"
    echo "  ./rviz.sh slam-opt          # Continue existing map"
    echo "  ./rviz.sh custom /path/to/my_config.rviz"
}

launch_rviz() {
    local config="$1"
    local extra_cmd="$2"

    echo "Launching RViz in ${MODE} mode..."
    echo "Using DISPLAY=${DISPLAY}"

    # Detect if we have a TTY
    local DOCKER_FLAGS="-i"
    if [ -t 0 ]; then
        DOCKER_FLAGS="-it"
    fi

    # Copy host's Xauthority file to container for X11 auth
    # Use /tmp/.Xauthority to avoid "device busy" issues with mounted /root/.Xauthority
    if [ -f "${HOME}/.Xauthority" ]; then
        docker cp "${HOME}/.Xauthority" ${CONTAINER_NAME}:/tmp/.Xauthority 2>/dev/null
    fi

    # Use software rendering for X11 forwarding (required for RPi)
    docker exec ${DOCKER_FLAGS} \
        -e DISPLAY=${DISPLAY} \
        -e XAUTHORITY=/tmp/.Xauthority \
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
        USE_EKF_SLAM=false

        # Check for ekf flag
        for arg in "$@"; do
            if [ "$arg" = "--ekf" ] || [ "$arg" = "ekf" ]; then
                USE_EKF_SLAM=true
            fi
        done

        if [ "$USE_EKF_SLAM" = true ]; then
            echo "Starting OPTIMIZED SLAM with EKF sensor fusion..."
            echo "  EKF fuses: RF2O LiDAR odometry + IMU"
        else
            echo "Starting OPTIMIZED SLAM with improved parameters..."
        fi
        echo ""
        echo "Using DISPLAY=${DISPLAY}"

        # Copy configs to container
        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

        # Always check for duplicates first
        cleanup_duplicate_nodes
        docker cp "${SCRIPT_DIR}/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml
        docker cp "${SCRIPT_DIR}/view_slam_2d.rviz" ${CONTAINER_NAME}:/tmp/view_slam_2d.rviz 2>/dev/null

        # Copy EKF configs if using EKF
        if [ "$USE_EKF_SLAM" = true ]; then
            docker cp "${SCRIPT_DIR}/bringup_ekf_simple.launch.py" ${CONTAINER_NAME}:/root/ugv_ws/install/ugv_bringup/share/ugv_bringup/launch/ 2>/dev/null
            # Also copy the updated ekf.yaml from ugv_ws
            docker cp /home/ws/ugv_ws/src/ugv_main/ugv_bringup/param/ekf.yaml ${CONTAINER_NAME}:/root/ugv_ws/install/ugv_bringup/share/ugv_bringup/param/ekf.yaml 2>/dev/null
        fi

        # Handle new_map option - restart container to clear zombie processes
        if [ "${NEW_MAP}" = "new_map" ] || [ "${NEW_MAP}" = "new" ] || [ "${NEW_MAP}" = "--new-map" ]; then
            echo ""
            echo "*** NEW MAP MODE - Restarting container for clean slate ***"
            echo ""
            echo "This restarts the container to clear zombie processes that cause drift."
            echo ""

            # Restart the container - this is the ONLY way to clear zombie processes
            # Killing processes leaves zombies which accumulate and cause odometry drift
            echo "Restarting container (clears all zombie processes)..."
            docker restart ${CONTAINER_NAME}

            # Wait for container to be fully up
            echo "Waiting for container to start..."
            sleep 5

            # Re-copy configs after restart
            docker cp "${SCRIPT_DIR}/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml
            docker cp "${SCRIPT_DIR}/view_slam_2d.rviz" ${CONTAINER_NAME}:/tmp/view_slam_2d.rviz 2>/dev/null
            docker cp "${SCRIPT_DIR}/ekf_lidar_imu.yaml" ${CONTAINER_NAME}:/tmp/ekf_lidar_imu.yaml 2>/dev/null

            # Start fresh bringup
            if [ "$USE_EKF_SLAM" = true ]; then
                echo "Starting robot bringup with EKF..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                source /root/ugv_ws/install/setup.bash && \
                export UGV_MODEL=ugv_beast && \
                export LDLIDAR_MODEL=ld19 && \
                ros2 launch ugv_bringup bringup_ekf_simple.launch.py > /tmp/bringup.log 2>&1
                "
                sleep 10  # EKF needs more time
            else
                echo "Starting robot bringup..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                source /root/ugv_ws/install/setup.bash && \
                export UGV_MODEL=ugv_beast && \
                export LDLIDAR_MODEL=ld19 && \
                ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true > /tmp/bringup.log 2>&1
                "
                sleep 6
            fi

            # Verify odometry is near zero
            echo "Checking odometry reset..."
            ODOM_CHECK=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /odom --once 2>/dev/null | grep -A2 'position:' | grep 'x:' | awk '{print \$2}'" 2>/dev/null)
            if [ -n "${ODOM_CHECK}" ]; then
                echo "  Odometry X: ${ODOM_CHECK}"
            else
                echo "  Odometry: waiting..."
            fi

            # Start fresh SLAM
            echo "Starting fresh SLAM with optimized config..."
            docker exec -d ${CONTAINER_NAME} /bin/bash -c "
            source /opt/ros/humble/setup.bash && \
            source /root/ugv_ws/install/setup.bash && \
            ros2 launch slam_toolbox online_async_launch.py \
              use_sim_time:=false \
              slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1
            "
            sleep 3

            # Verify no zombie processes
            ZOMBIES=$(docker exec ${CONTAINER_NAME} ps aux 2>/dev/null | grep -c "<defunct>" || echo "0")
            if [ "${ZOMBIES}" -gt 0 ]; then
                echo "Warning: ${ZOMBIES} zombie processes found (unexpected after restart)"
            else
                echo "Clean start - no zombie processes!"
            fi

            echo ""
            echo "Fresh map started!"
        else
            # Normal mode - check if already running
            # Check if bringup is already running (check both ekf and non-ekf)
            BRINGUP_RUNNING=$(docker exec ${CONTAINER_NAME} pgrep -f "ugv_bringup" 2>/dev/null || true)
            if [ -z "$BRINGUP_RUNNING" ]; then
                if [ "$USE_EKF_SLAM" = true ]; then
                    echo "Starting robot bringup with EKF..."
                    docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                    source /opt/ros/humble/setup.bash && \
                    source /root/ugv_ws/install/setup.bash && \
                    export UGV_MODEL=ugv_beast && \
                    export LDLIDAR_MODEL=ld19 && \
                    ros2 launch ugv_bringup bringup_ekf_simple.launch.py > /tmp/bringup.log 2>&1
                    "
                    sleep 10
                else
                    echo "Starting robot bringup..."
                    docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                    source /opt/ros/humble/setup.bash && \
                    source /root/ugv_ws/install/setup.bash && \
                    export UGV_MODEL=ugv_beast && \
                    export LDLIDAR_MODEL=ld19 && \
                    ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true > /tmp/bringup.log 2>&1
                    "
                    sleep 5
                fi
            else
                echo "Bringup already running, skipping..."
                # Check if EKF is running when requested
                if [ "$USE_EKF_SLAM" = true ]; then
                    EKF_RUNNING=$(docker exec ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null | grep -c ekf_filter_node" 2>/dev/null || echo "0")
                    if [ "$EKF_RUNNING" = "0" ]; then
                        echo "  NOTE: EKF not running. Use 'new_map --ekf' to start fresh with EKF."
                    else
                        echo "  EKF: RUNNING"
                    fi
                fi
            fi

            # Check if SLAM is already running
            if ! docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
                echo "Starting SLAM with optimized config..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                source /root/ugv_ws/install/setup.bash && \
                ros2 launch slam_toolbox online_async_launch.py \
                  use_sim_time:=false \
                  slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1
                "
                sleep 3
            else
                echo "SLAM already running, skipping..."
            fi
        fi

        echo "Launching RViz with manufacturer's SLAM config..."
        launch_rviz "/tmp/view_slam_2d.rviz"
        ;;
    slam-ekf)
        NEW_MAP="$2"

        echo "Starting SLAM with EKF triple sensor fusion (BEST ACCURACY)..."
        echo ""
        echo "This uses:"
        echo "  - Wheel encoders: velocity (knows when robot is actually moving)"
        echo "  - LiDAR odometry: position correction"
        echo "  - IMU: orientation and angular velocity"
        echo ""
        echo "Using DISPLAY=${DISPLAY}"

        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

        # Handle new_map option - restart container to clear zombie processes
        if [ "${NEW_MAP}" = "new_map" ] || [ "${NEW_MAP}" = "new" ] || [ "${NEW_MAP}" = "--new-map" ]; then
            echo ""
            echo "*** NEW MAP MODE - Restarting container for clean slate ***"
            echo ""

            # Restart container for clean slate
            echo "Restarting container..."
            docker restart ${CONTAINER_NAME}
            sleep 5

            # Copy configs
            docker cp "${SCRIPT_DIR}/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml
            docker cp "${SCRIPT_DIR}/ekf_wheel_lidar_imu.yaml" ${CONTAINER_NAME}:/tmp/ekf_wheel_lidar_imu.yaml
            docker cp "${SCRIPT_DIR}/view_slam_2d.rviz" ${CONTAINER_NAME}:/tmp/view_slam_2d.rviz 2>/dev/null

            # Start bringup WITHOUT odom TF (EKF will publish it)
            echo "Starting robot bringup (EKF handles odometry TF)..."
            docker exec -d ${CONTAINER_NAME} /bin/bash -c "
            source /opt/ros/humble/setup.bash && \
            source /root/ugv_ws/install/setup.bash && \
            export UGV_MODEL=ugv_beast && \
            export LDLIDAR_MODEL=ld19 && \
            ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=false > /tmp/bringup.log 2>&1
            "
            sleep 6

            # Check bringup
            if docker exec ${CONTAINER_NAME} pgrep -f "bringup_lidar" > /dev/null 2>&1; then
                echo "  Bringup: RUNNING"
            else
                echo "  Bringup: FAILED"
            fi

            # Start EKF fusion
            echo "Starting EKF fusion (wheel + LiDAR + IMU)..."
            docker exec -d ${CONTAINER_NAME} /bin/bash -c "
            source /opt/ros/humble/setup.bash && \
            source /root/ugv_ws/install/setup.bash && \
            ros2 run robot_localization ekf_node \
              --ros-args \
              --params-file /tmp/ekf_wheel_lidar_imu.yaml \
              -r /odometry/filtered:=/odom_fused > /tmp/ekf.log 2>&1
            "
            sleep 3

            # Check EKF
            if docker exec ${CONTAINER_NAME} pgrep -f "ekf_node" > /dev/null 2>&1; then
                echo "  EKF: RUNNING"
            else
                echo "  EKF: FAILED - check /tmp/ekf.log"
            fi

            # Start SLAM
            echo "Starting SLAM..."
            docker exec -d ${CONTAINER_NAME} /bin/bash -c "
            source /opt/ros/humble/setup.bash && \
            source /root/ugv_ws/install/setup.bash && \
            ros2 launch slam_toolbox online_async_launch.py \
              use_sim_time:=false \
              slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1
            "
            sleep 3

            # Check SLAM
            if docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
                echo "  SLAM: RUNNING"
            else
                echo "  SLAM: FAILED"
            fi

            # Verify no zombie processes
            ZOMBIES=$(docker exec ${CONTAINER_NAME} ps aux 2>/dev/null | grep -c "<defunct>" || echo "0")
            if [ "${ZOMBIES}" -gt 0 ]; then
                echo "Warning: ${ZOMBIES} zombie processes found"
            else
                echo "Clean start - no zombie processes!"
            fi
        else
            # Normal mode - continue existing session or start if not running
            # Copy configs
            docker cp "${SCRIPT_DIR}/slam_toolbox_optimized.yaml" ${CONTAINER_NAME}:/tmp/slam_toolbox_optimized.yaml
            docker cp "${SCRIPT_DIR}/ekf_wheel_lidar_imu.yaml" ${CONTAINER_NAME}:/tmp/ekf_wheel_lidar_imu.yaml
            docker cp "${SCRIPT_DIR}/view_slam_2d.rviz" ${CONTAINER_NAME}:/tmp/view_slam_2d.rviz 2>/dev/null

            # Check if bringup is already running
            if ! docker exec ${CONTAINER_NAME} pgrep -f "bringup_lidar" > /dev/null 2>&1; then
                echo "Starting robot bringup (EKF handles odometry TF)..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                source /root/ugv_ws/install/setup.bash && \
                export UGV_MODEL=ugv_beast && \
                export LDLIDAR_MODEL=ld19 && \
                ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=false > /tmp/bringup.log 2>&1
                "
                sleep 6
            else
                echo "Bringup already running, skipping..."
            fi

            # Check if EKF is already running
            if ! docker exec ${CONTAINER_NAME} pgrep -f "ekf_node" > /dev/null 2>&1; then
                echo "Starting EKF fusion (wheel + LiDAR + IMU)..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                source /root/ugv_ws/install/setup.bash && \
                ros2 run robot_localization ekf_node \
                  --ros-args \
                  --params-file /tmp/ekf_wheel_lidar_imu.yaml \
                  -r /odometry/filtered:=/odom_fused > /tmp/ekf.log 2>&1
                "
                sleep 3
            else
                echo "EKF already running, skipping..."
            fi

            # Check if SLAM is already running
            if ! docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
                echo "Starting SLAM..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                source /root/ugv_ws/install/setup.bash && \
                ros2 launch slam_toolbox online_async_launch.py \
                  use_sim_time:=false \
                  slam_params_file:=/tmp/slam_toolbox_optimized.yaml > /tmp/slam.log 2>&1
                "
                sleep 3
            else
                echo "SLAM already running, skipping..."
            fi
        fi

        echo ""
        echo "EKF fusion chain:"
        echo "  Wheel + LiDAR + IMU -> EKF -> odom TF -> SLAM"
        echo ""
        echo "Launching RViz..."
        launch_rviz "/tmp/view_slam_2d.rviz"
        ;;
    slam-carto|slam-cartographer)
        NEW_MAP="$2"

        echo "Starting Google Cartographer SLAM (BEST ACCURACY)..."
        echo ""
        echo "Cartographer advantages:"
        echo "  - Scan-to-submap matching (less reliant on odometry)"
        echo "  - Global pose graph optimization"
        echo "  - Better loop closure detection"
        echo ""
        echo "Using DISPLAY=${DISPLAY}"

        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

        # Always check for duplicates first
        cleanup_duplicate_nodes

        # Handle new_map option
        if [ "${NEW_MAP}" = "new_map" ] || [ "${NEW_MAP}" = "new" ] || [ "${NEW_MAP}" = "--new-map" ]; then
            echo ""
            echo "*** NEW MAP MODE - Restarting for clean slate ***"
            echo ""

            docker restart ${CONTAINER_NAME}
            sleep 5

            # Copy configs
            docker cp "${SCRIPT_DIR}/cartographer_2d.lua" ${CONTAINER_NAME}:/tmp/cartographer_2d.lua
            docker cp "${SCRIPT_DIR}/view_slam_2d.rviz" ${CONTAINER_NAME}:/tmp/view_slam_2d.rviz 2>/dev/null

            # Start bringup
            echo "Starting robot bringup..."
            docker exec -d ${CONTAINER_NAME} /bin/bash -c "
            source /opt/ros/humble/setup.bash && \
            source /root/ugv_ws/install/setup.bash && \
            export UGV_MODEL=ugv_beast && \
            export LDLIDAR_MODEL=ld19 && \
            ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true > /tmp/bringup.log 2>&1
            "
            sleep 6

            if docker exec ${CONTAINER_NAME} pgrep -f "bringup_lidar" > /dev/null 2>&1; then
                echo "  Bringup: RUNNING"
            else
                echo "  Bringup: FAILED"
            fi

            # Start Cartographer
            echo "Starting Cartographer..."
            docker exec -d ${CONTAINER_NAME} /bin/bash -c "
            source /opt/ros/humble/setup.bash && \
            ros2 run cartographer_ros cartographer_node \
              -configuration_directory /tmp \
              -configuration_basename cartographer_2d.lua > /tmp/cartographer.log 2>&1
            "
            sleep 3

            if docker exec ${CONTAINER_NAME} pgrep -f "cartographer_node" > /dev/null 2>&1; then
                echo "  Cartographer: RUNNING"
            else
                echo "  Cartographer: FAILED - check /tmp/cartographer.log"
                docker exec ${CONTAINER_NAME} cat /tmp/cartographer.log 2>/dev/null | tail -10
            fi

            # Start occupancy grid node
            echo "Starting occupancy grid publisher..."
            docker exec -d ${CONTAINER_NAME} /bin/bash -c "
            source /opt/ros/humble/setup.bash && \
            ros2 run cartographer_ros cartographer_occupancy_grid_node \
              -resolution 0.03 \
              -publish_period_sec 1.0 > /tmp/occupancy_grid.log 2>&1
            "
            sleep 2

            if docker exec ${CONTAINER_NAME} pgrep -f "occupancy_grid" > /dev/null 2>&1; then
                echo "  Occupancy grid: RUNNING"
            else
                echo "  Occupancy grid: FAILED"
            fi

            # Verify clean
            ZOMBIES=$(docker exec ${CONTAINER_NAME} ps aux 2>/dev/null | grep -c "<defunct>" || echo "0")
            if [ "${ZOMBIES}" -gt 0 ]; then
                echo "Warning: ${ZOMBIES} zombie processes"
            else
                echo "Clean start!"
            fi
        else
            # Continue mode
            docker cp "${SCRIPT_DIR}/cartographer_2d.lua" ${CONTAINER_NAME}:/tmp/cartographer_2d.lua
            docker cp "${SCRIPT_DIR}/view_slam_2d.rviz" ${CONTAINER_NAME}:/tmp/view_slam_2d.rviz 2>/dev/null

            if ! docker exec ${CONTAINER_NAME} pgrep -f "bringup_lidar" > /dev/null 2>&1; then
                echo "Starting robot bringup..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                source /root/ugv_ws/install/setup.bash && \
                export UGV_MODEL=ugv_beast && \
                export LDLIDAR_MODEL=ld19 && \
                ros2 launch ugv_bringup bringup_lidar.launch.py pub_odom_tf:=true > /tmp/bringup.log 2>&1
                "
                sleep 6
            else
                echo "Bringup already running..."
            fi

            # Kill slam_toolbox if running
            if docker exec ${CONTAINER_NAME} pgrep -f "slam_toolbox" > /dev/null 2>&1; then
                echo "Stopping slam_toolbox..."
                docker exec ${CONTAINER_NAME} pkill -f "slam_toolbox" 2>/dev/null
                sleep 2
            fi

            if ! docker exec ${CONTAINER_NAME} pgrep -f "cartographer_node" > /dev/null 2>&1; then
                echo "Starting Cartographer..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                ros2 run cartographer_ros cartographer_node \
                  -configuration_directory /tmp \
                  -configuration_basename cartographer_2d.lua > /tmp/cartographer.log 2>&1
                "
                sleep 3
            else
                echo "Cartographer already running..."
            fi

            if ! docker exec ${CONTAINER_NAME} pgrep -f "occupancy_grid" > /dev/null 2>&1; then
                echo "Starting occupancy grid publisher..."
                docker exec -d ${CONTAINER_NAME} /bin/bash -c "
                source /opt/ros/humble/setup.bash && \
                ros2 run cartographer_ros cartographer_occupancy_grid_node \
                  -resolution 0.03 \
                  -publish_period_sec 1.0 > /tmp/occupancy_grid.log 2>&1
                "
                sleep 2
            else
                echo "Occupancy grid already running..."
            fi
        fi

        echo ""
        echo "Cartographer SLAM running!"
        echo "  /map - Occupancy grid"
        echo "  /submap_list - Submaps"
        echo ""
        echo "Launching RViz..."
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
