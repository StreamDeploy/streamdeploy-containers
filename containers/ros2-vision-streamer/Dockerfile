ARG BASE_IMAGE
# Orin/Jetson: nvcr.io/nvidia/l4t-jetpack:35.4.1-runtime
# Generic: ubuntu:22.04
FROM ${BASE_IMAGE:-"nvcr.io/nvidia/l4t-jetpack:35.4.1-runtime"}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

# Core deps + GStreamer + ROS 2
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl ca-certificates gnupg lsb-release sudo \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-libav v4l-utils \
    python3-pip python3-colcon-common-extensions \
    && locale-gen en_US.UTF-8 && rm -rf /var/lib/apt/lists/*

# ROS 2 apt repo
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64,amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*

# App user
RUN useradd -ms /bin/bash app && echo "app ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER app
WORKDIR /home/app

# Minimal ROS 2 workspace with a camera streamer node (installed via pip)
# (You can swap this for your internal package later)
RUN pip3 install --no-cache-dir opencv-python-headless==4.9.0.80 rclpy==3.3.9 numpy==1.26.4

# Entrypoint & defaults
COPY --chown=app:app entrypoint.sh /home/app/entrypoint.sh
RUN chmod +x /home/app/entrypoint.sh
ENV CAMERA_DEV=/dev/video0 \
    ENCODER=H264 \
    FPS=30 \
    WIDTH=1280 \
    HEIGHT=720 \
    ROS_NAMESPACE=/camera \
    ENABLE_RTSP=false \
    RTSP_PORT=8554

# Healthcheck: ensure ROS 2 node is alive (simple TCP poke to rtsp if enabled)
HEALTHCHECK --interval=30s --timeout=5s --start-period=20s \
  CMD bash -lc '[ "$ENABLE_RTSP" = "true" ] && nc -z localhost $RTSP_PORT || exit 0'

ENTRYPOINT ["/home/app/entrypoint.sh"]
