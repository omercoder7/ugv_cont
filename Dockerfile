# Clean Dockerfile for UGV Beast on ROS 2 Humble (ARM64 Optimized)
# Uses the stable, official ROS core image for ARM64 based on Ubuntu Jammy
FROM ros:humble-ros-core-jammy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV UCF_FORCE_CONFFOLD=1

# Define non-root user and workspace path
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV UGV_WS_DIR=/home/${USERNAME}/ugv_ws

# --- 1. Repository Setup and Dependencies ---
RUN echo "--> Setting up OSRF repositories and dependencies..." && \
    apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release wget sudo software-properties-common git build-essential cmake \
    # Add the OSRF (Ignition Gazebo) repository key
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkg-osrfoundation-archive-keyring.gpg && \
    # Add OSRF repository source for Ignition packages (native binaries)
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkg-osrfoundation-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# --- 2. Core ROS/System Installation ---
RUN echo "--> Installing core components and build tools..." && \
    apt-get update && \
    # Install full desktop/Rviz, build tools, and Python dependencies
    apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rosbridge-suite \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    # Core Libraries needed for compilation
    libeigen3-dev \
    libxml2-dev \
    # Install Ignition Fortress (Gazebo for ROS 2 Humble on Jammy)
    ignition-fortress \
    # Cleanup
    && apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/*

# --- 3. User Setup and Entrypoint ---
# Create non-root user 'ros' to match ownership on host machine (UID 1000 is default)
RUN groupadd --gid $USER_GID $USERNAME || true && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG sudo $USERNAME && \
    # Ensure user can use APT for rosdep installation later if needed
    echo $USERNAME ALL=\(root\) NOPASSWD: /usr/bin/apt-get > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Copy entrypoint and make executable
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Initialize rosdep as root before switching to non-root user
RUN rosdep init || true

# Switch to the non-root user for security and file ownership
USER $USERNAME
WORKDIR /home/$USERNAME

# --- 4. Workspace Preparation ---
RUN mkdir -p ${UGV_WS_DIR}/src
WORKDIR ${UGV_WS_DIR}

# Clone repository and install Python dependencies
COPY requirements.txt ./
RUN git clone -b ros2-humble-develop https://github.com/waveshareteam/ugv_ws.git src/ugv_ws_original && \
    python3 -m pip install -U pip "setuptools<81" && \
    python3 -m pip install -r requirements.txt

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
