ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"ubuntu:22.04"}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl ca-certificates gnupg lsb-release udev \
    && locale-gen en_US.UTF-8

# ROS 2 repo
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64,amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-realsense2-camera \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# udev rules for RealSense (mounted persistently is recommended too)
RUN mkdir -p /etc/udev/rules.d && \
    echo 'ATTRS{idVendor}=="8086", MODE:="0666"' > /etc/udev/rules.d/99-realsense-libusb.rules

RUN useradd -ms /bin/bash app && usermod -aG plugdev app
USER app
WORKDIR /home/app

COPY --chown=app:app entrypoint.sh /home/app/entrypoint.sh
RUN chmod +x /home/app/entrypoint.sh

ENV SERIAL="" \
    ENABLE_SYNC=false \
    COLOR_WIDTH=1280 COLOR_HEIGHT=720 COLOR_FPS=30 \
    DEPTH_WIDTH=640 DEPTH_HEIGHT=480 DEPTH_FPS=30

HEALTHCHECK --interval=30s --timeout=5s --start-period=20s \
  CMD bash -lc 'ros2 topic list | grep -q "/camera/color/image_raw"'

ENTRYPOINT ["/home/app/entrypoint.sh"]
