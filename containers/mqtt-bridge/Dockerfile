ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"ubuntu:22.04"}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip curl ca-certificates gnupg lsb-release \
    && rm -rf /var/lib/apt/lists/*

# ROS 2
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64,amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir paho-mqtt==1.6.1

WORKDIR /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

ENV MQTT_HOST=mqtt \
    MQTT_PORT=1883 \
    MQTT_TLS=false \
    MQTT_USERNAME= \
    MQTT_PASSWORD= \
    ROS_SUBSCRIBE=/chatter \
    MQTT_PUB_TOPIC=robot/chatter \
    MQTT_SUB_TOPIC=robot/cmd \
    ROS_PUBLISH=/cmd \
    QOS=0

HEALTHCHECK --interval=30s --timeout=5s --start-period=15s \
  CMD bash -lc 'ros2 topic list | grep -q "${ROS_SUBSCRIBE}" || exit 0'

ENTRYPOINT ["/app/entrypoint.sh"]
