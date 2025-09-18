ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"ubuntu:22.04"}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl ca-certificates gnupg lsb-release python3-pip \
    && locale-gen en_US.UTF-8

# ROS 2
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64,arm64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Python deps (CPU-friendly; will run with CUDA if libs are present)
RUN pip3 install --no-cache-dir \
    onnxruntime==1.18.0 numpy==1.26.4 pillow==10.3.0 opencv-python-headless==4.9.0.80

WORKDIR /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

HEALTHCHECK --interval=30s --timeout=8s --start-period=25s \
  CMD bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 topic list | grep -q /perception/detections"

ENTRYPOINT ["/app/entrypoint.sh"]
