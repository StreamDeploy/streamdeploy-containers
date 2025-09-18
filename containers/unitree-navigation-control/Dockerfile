ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"ubuntu:22.04"}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl ca-certificates gnupg lsb-release \
    && locale-gen en_US.UTF-8

# ROS 2 + Nav2
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64,arm64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-geometry-msgs \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -ms /bin/bash app && echo "app ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER app
WORKDIR /home/app

COPY --chown=app:app entrypoint.sh /home/app/entrypoint.sh
RUN chmod +x /home/app/entrypoint.sh

HEALTHCHECK --interval=30s --timeout=8s --start-period=25s \
  CMD bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 topic list | grep -q /unitree/gait_cmd"

ENTRYPOINT ["/home/app/entrypoint.sh"]
