# ──────────────────────────────────────────────────────────────────────────────
# All critical versions pinned. See docs/drone_follow_setup.md §3.
#   PX4-Autopilot    v1.14.0
#   px4_msgs         release/1.14  ← MUST match PX4; silent DDS failure if wrong
#   px4_ros_com      release/1.14
#   Micro-XRCE-DDS   v2.4.3  (v2.4.2 unbuildable: FastDDS 2.12.x branch deleted)
#   empy             3.3.4         ← newer breaks colcon build of px4_ros_com
#   ros_gz           ros-humble-ros-gzgarden  (Garden, not Fortress)
#   numpy            <2            ← ultralytics 8.x requires numpy <2
#   ultralytics      8.3.0
# ──────────────────────────────────────────────────────────────────────────────
FROM osrf/ros:humble-desktop-full

# Install system build dependencies
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    clang \
    lldb \
    ninja-build \
    libgtest-dev \
    libeigen3-dev \
    libopencv-dev \
    libyaml-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-tools \
    sudo \
    wget \
    curl \
    tmux \
    ruby \
    tmuxinator \
  && rm -rf /var/lib/apt/lists/*

# Pin empy and setuptools BEFORE any colcon builds — newer empy breaks px4_ros_com
RUN pip3 install "empy==3.3.4" "setuptools==58.2.0"

# Install PX4 — pinned to v1.14.0
RUN cd /root && \
    git clone --branch v1.14.0 --depth 1 --recursive \
        https://github.com/PX4/PX4-Autopilot.git && \
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh && \
    cd PX4-Autopilot && \
    make px4_sitl

# Setup Micro XRCE-DDS Agent — pinned to v2.4.2
RUN cd /root && \
    git clone --branch v2.4.3 --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && make && make install && \
    ldconfig /usr/local/lib/

# Build ROS 2 Workspace ws_sensor_combined
# NOTE: px4_msgs uses release/1.14; px4_ros_com uses release/v1.14 (different naming)
RUN mkdir -p /root/ws_sensor_combined/src && \
    cd /root/ws_sensor_combined/src && \
    git clone --branch release/1.14 --depth 1 \
        https://github.com/PX4/px4_msgs.git && \
    git clone --branch release/v1.14 --depth 1 \
        https://github.com/PX4/px4_ros_com.git && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  cd /root/ws_sensor_combined && colcon build"

# Build ROS 2 Workspace ws_offboard_control (same version pins)
RUN mkdir -p /root/ws_offboard_control/src && \
    cd /root/ws_offboard_control/src && \
    git clone --branch release/1.14 --depth 1 \
        https://github.com/PX4/px4_msgs.git && \
    git clone --branch release/v1.14 --depth 1 \
        https://github.com/PX4/px4_ros_com.git && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  cd /root/ws_offboard_control && colcon build"

# ros_gz Garden bridge — BUILT FROM SOURCE. Do not "fix" this back to an
# apt package without reading this whole comment first.
#
# ros-humble-ros-gzgarden (the distro-suffixed apt package this used to
# install) has been REMOVED from packages.ros.org entirely (confirmed via
# `apt-cache madison ros-humble-ros-gzgarden` returning zero results — not
# superseded, just gone). osrf/ros:humble-desktop-full now ships a
# same-named-downstream but DIFFERENT replacement,
# ros-humble-ros-gz-{bridge,sim,image,interfaces}, pre-installed. It looks
# like a drop-in (same ros_gz_bridge/ros_gz_sim package names) but is NOT:
# its parameter_bridge binary links against libignition-transport11
# (Fortress-era), while gz-garden (installed above via PX4's own setup
# script) is libgz-transport12. These are incompatible major versions of
# the transport library — the bridge process cannot discover or exchange
# messages with the simulator AT ALL. Symptom: `gz topic echo` shows data
# fine, Gazebo renders fine, but `ros2 topic echo /camera` (or any bridged
# topic) shows nothing, ever. Confirmed by hand with `ldd` on both
# binaries before writing this.
#
# Setting GZ_VERSION=garden as a Dockerfile ENV var does NOT fix this —
# that only affects ros_gz's *source* package.xml conditional
# dependencies at build time; it has zero effect on an apt package the
# ROS buildfarm already compiled with its own GZ_VERSION. Confirmed the
# hard way. The only real fix is compiling ros_gz ourselves against the
# gz-garden dev packages that are actually installed.
RUN mkdir -p /root/ros_gz_ws/src && \
    cd /root/ros_gz_ws/src && \
    git clone -b caguero/humble_garden --depth 1 \
        https://github.com/gazebosim/ros_gz.git && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  export GZ_VERSION=garden && \
                  cd /root/ros_gz_ws && \
                  colcon build --packages-select ros_gz_interfaces ros_gz_bridge"

# Python runtime deps — pinned.
# If you don't have a GPU, uncomment the CPU-only torch line below instead:
# RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install \
    "mavsdk==1.4.9" \
    "aioconsole==0.6.1" \
    "pygame==2.5.2" \
    "opencv-python==4.9.0.80" \
    "numpy<2" \
    "ultralytics==8.3.0"

# Copy models and worlds from local repository
RUN mkdir -p /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY . /root/PX4-ROS2-Gazebo-YOLOv8
COPY models/. /root/.gz/models/
COPY models_docker/. /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY worlds/default_docker.sdf /root/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf

# Pre-fetch the OakD-Lite camera model so the x500_depth spawn does not
# depend on (and flakily time out on) a fuel.gazebosim.org download at
# first sim start.
RUN gz fuel download -u https://fuel.gazebosim.org/1.0/RudisLaboratories/models/OakD-Lite

# Setup gimbal joints for camera control
RUN python3 /root/PX4-ROS2-Gazebo-YOLOv8/setup_gimbal.py

# Shell setup
# ros_gz_ws sourced LAST so its from-source ros_gz_bridge/ros_gz_interfaces
# overlay/shadow the broken apt ros-humble-ros-gz-* packages baked into
# the base image (see the long comment above the ros_gz build step).
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws_sensor_combined/install/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros_gz_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export GZ_VERSION=garden" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/root/.gz/models" >> /root/.bashrc && \
    echo "export PATH=\$PATH:/root/.local/bin" >> /root/.bashrc

# Copy tmuxinator configuration
COPY px4_ros2_gazebo.yml /root/.config/tmuxinator/px4_ros2_gazebo.yml

# Set default command to start tmuxinator
CMD ["tmuxinator", "start", "px4_ros2_gazebo"]
