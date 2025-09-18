ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"ubuntu:22.04"}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl ca-certificates gnupg lsb-release sudo \
    python3-pip python3-colcon-common-extensions \
    net-tools iproute2 iputils-ping \
    && locale-gen en_US.UTF-8

# ROS 2 apt repo
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-diagnostic-updater \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-geometry-msgs \
      ros-${ROS_DISTRO}-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Non-root user
RUN useradd -ms /bin/bash robot && echo "robot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER robot
WORKDIR /home/robot

# Workspace mount points (bind your SDK/bringup here)
RUN mkdir -p /work/ros_ws/src
ENV UNITREE_SDK_PATH=/work/unitree \
    BRINGUP_PACKAGE=unitree_bringup \
    BRINGUP_LAUNCH=humanoid_bringup.launch.py

COPY --chown=robot:robot entrypoint.sh /home/robot/entrypoint.sh
RUN chmod +x /home/robot/entrypoint.sh

HEALTHCHECK --interval=30s --timeout=10s --start-period=30s \
  CMD bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 node list | grep -qi unitree || exit 1"

ENTRYPOINT ["/home/robot/entrypoint.sh"]
