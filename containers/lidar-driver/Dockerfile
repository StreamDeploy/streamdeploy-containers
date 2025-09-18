ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"ubuntu:22.04"}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl ca-certificates gnupg lsb-release \
    && locale-gen en_US.UTF-8

# ROS 2 repo + drivers
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64,amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-velodyne \
      ros-${ROS_DISTRO}-velodyne-pointcloud \
      ros-${ROS_DISTRO}-rplidar-ros \
      ros-${ROS_DISTRO}-rviz2 \
      python3-flask \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -ms /bin/bash app
USER app
WORKDIR /home/app

COPY --chown=app:app entrypoint.sh /home/app/entrypoint.sh
RUN chmod +x /home/app/entrypoint.sh

ENV LIDAR_VENDOR="velodyne" \
    DEVICE_PORT="/dev/ttyUSB0" \
    ETHERNET_IP="192.168.1.201" \
    FRAME_ID="laser" \
    SNAPSHOT_SERVER=false \
    RVIZ=false

HEALTHCHECK --interval=30s --timeout=5s --start-period=20s \
  CMD bash -lc 'ros2 topic list | grep -q "/scan" || ros2 topic list | grep -q "/points"'

ENTRYPOINT ["/home/app/entrypoint.sh"]
