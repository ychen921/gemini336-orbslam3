FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=30
ENV ROS_LOCALHOST_ONLY=0

# ------------------------------------------------------------
# Base development environment
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    git \
    curl \
    wget \
    vim \
    htop \
    tree \
    tmux \
    less \
    unzip \
    zip \
    pkg-config \
    software-properties-common \
    iproute2 \
    net-tools \
    usbutils \
    udev \
    mesa-utils \
    python3 \
    python3-pip \
    python3-numpy \
    python3-argcomplete \
    python3-colcon-common-extensions \
    \
    # ORB-SLAM3 / Eigen / OpenCV
    libeigen3-dev \
    libopencv-dev \
    \
    # Pangolin required / recommended dependencies
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libepoxy-dev \
    libglew-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libjpeg-dev \
    libpng-dev \
    libopenexr-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libavdevice-dev \
    \
    # OrbbecSDK_ROS2 common dependencies
    libgflags-dev \
    libgoogle-glog-dev \
    nlohmann-json3-dev \
    libdw-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    \
    # ROS 2 core
    ros-humble-rclcpp \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-diagnostic-msgs \
    ros-humble-statistics-msgs \
    \
    # ROS 2 TF
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs \
    ros-humble-tf2-geometry-msgs \
    \
    # ROS 2 image / camera
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-image-publisher \
    ros-humble-camera-info-manager \
    ros-humble-message-filters \
    \
    # ROS 2 launch / diagnostics / bag
    ros-humble-diagnostic-updater \
    ros-humble-xacro \
    ros-humble-backward-ros \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2-compression-zstd \
    ros-humble-rosbag2-storage-mcap \
    \
    # DDS
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Shell setup
#
# Pangolin and ORB-SLAM3 source code are intentionally NOT cloned here.
# Keep them in the host workspace and mount the workspace into the
# container so the same source tree can be edited/traced from the host.
# ------------------------------------------------------------
SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-30}" >> /root/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=\${ROS_LOCALHOST_ONLY:-0}" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" >> /root/.bashrc

WORKDIR /workspaces/gemini336-orbslam3

CMD ["bash"]
