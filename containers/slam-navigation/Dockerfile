ARG BASE_IMAGE
# Jetson-friendly base is fine; also runs on generic arm64
FROM ${BASE_IMAGE:-"ubuntu:22.04"}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl ca-certificates gnupg lsb-release sudo \
    python3-pip python3-colcon-common-extensions \
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
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-slam-toolbox \
      ros-${ROS_DISTRO}-cartographer \
      ros-${ROS_DISTRO}-cartographer-ros \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-tf2-tools \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# App user
RUN useradd -ms /bin/bash app && echo "app ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER app
WORKDIR /home/app

COPY --chown=app:app entrypoint.sh /home/app/entrypoint.sh
RUN chmod +x /home/app/entrypoint.sh

ENV NAV_STACK="nav2" \
    SLAM_STACK="slam_toolbox" \
    MAP_TOPIC=/map \
    ODOM_TOPIC=/odom \
    SCAN_TOPIC=/scan \
    USE_SIM_TIME=false

HEALTHCHECK --interval=30s --timeout=5s --start-period=20s \
  CMD bash -lc 'ros2 node list | grep -q "nav2" || exit 1'

ENTRYPOINT ["/home/app/entrypoint.sh"]
