# Clean Dockerfile for UGV Beast on ROS 2 Humble (ARM64 Optimized)
# Uses the stable, official ROS core image for ARM64 based on Ubuntu Jammy
FROM ros:humble-ros-core-jammy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV UCF_FORCE_CONFFOLD=1
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Define workspace path (running as root)
ENV UGV_WS_DIR=/root/ugv_ws

# --- 1. Repository Setup and Dependencies ---
# Install base tools, then add OSRF repository for Ignition Gazebo
RUN echo "--> Setting up OSRF repositories and dependencies..." && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        apt-utils \
        curl \
        gnupg2 \
        lsb-release \
        wget \
        sudo \
        software-properties-common \
        git \
        build-essential \
        cmake \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkg-osrfoundation-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkg-osrfoundation-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# --- 2. Core ROS/System Installation ---
# Install ROS desktop, navigation, build tools, colcon, and Ignition Fortress
RUN echo "--> Installing core components and build tools..." && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-desktop \
        ros-humble-navigation2 \
        ros-humble-joint-state-publisher-gui \
        ros-humble-rosbridge-suite \
        python3-pip \
        python3-rosdep \
        python3-colcon-common-extensions \
        libeigen3-dev \
        libxml2-dev \
        ignition-fortress \
    && apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 3. Entrypoint Setup ---
# Copy entrypoint and make executable
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Initialize rosdep
RUN rosdep init || true

# Run as root for hardware access
WORKDIR /root

# --- 4. Workspace Preparation ---
RUN mkdir -p ${UGV_WS_DIR}/src
WORKDIR ${UGV_WS_DIR}

# Clone repository and install Python dependencies
COPY requirements.txt ./
RUN git clone -b ros2-humble-develop https://github.com/waveshareteam/ugv_ws.git src/ugv_ws_original && \
    python3 -m pip install -U pip && \
    python3 -m pip install --ignore-installed -r requirements.txt

# --- 5. Final Build ---
RUN echo "--> Building UGV workspace..." && \
    source /opt/ros/humble/setup.bash && \
    # Update rosdep (already initialized as root earlier)
    rosdep update && \
    # Install remaining dependencies required by the source packages
    rosdep install -i --from-path src --rosdistro humble -y --skip-keys "roslaunch cmake_modules catkin gazebo python3-flask libg2o ros-humble-libg2o nav2-bringup nav2_bringup ros-humble-nav2-bringup" && \
    # Build the entire workspace
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Set Entrypoint and Default Command
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
