ARG BASE_IMAGE
# Jetson-friendly by default; works on generic arm64 with CPU fallback
FROM ${BASE_IMAGE:-"nvcr.io/nvidia/l4t-pytorch:r35.4.1-py3"}

ENV DEBIAN_FRONTEND=noninteractive LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev build-essential curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    fastapi==0.111.0 uvicorn[standard]==0.30.0 pillow==10.3.0 numpy==1.26.4 \
    onnx==1.16.0 onnxruntime-gpu==1.18.0 onnxruntime==1.18.0

# Optional ROS 2 mode
ENV ROS_DISTRO=humble
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg lsb-release && \
    mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=arm64,amd64 signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

ENV MODEL_PATH=/models/yolov8.onnx \
    MODE=rest \
    SERVICE_HOST=0.0.0.0 SERVICE_PORT=8080 \
    ROS_TOPIC=/detections

EXPOSE 8080
HEALTHCHECK --interval=30s --timeout=5s --start-period=20s \
  CMD bash -lc '[ "$MODE" = "rest" ] && curl -fsS http://localhost:8080/health || exit 0'

ENTRYPOINT ["/app/entrypoint.sh"]
