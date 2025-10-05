ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}
USER root

# Install perception dependencies and tools
RUN apt-get update && apt-get install -y \
    bash \
    ca-certificates \
    netcat-traditional \
    python3-pip \
    python3-opencv \
    ros-humble-vision-opencv \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-rqt-image-view \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python perception libraries
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    pillow \
    torch \
    torchvision \
    ultralytics

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Environment variables for configuration
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedx_cpp \
    MODEL_PATH="/opt/openmind/models" \
    DETECTION_MODEL="yolov8n.pt" \
    INPUT_TOPIC="/camera/image_raw" \
    OUTPUT_TOPIC="/perception/detections" \
    CONFIDENCE_THRESHOLD="0.5"

# Create model directory
RUN mkdir -p /opt/openmind/models

# Expose common ROS 2 ports
EXPOSE 11511

HEALTHCHECK --interval=30s --timeout=10s --start-period=30s \
    CMD bash -lc 'source /opt/ros/humble/setup.bash && ros2 node list | grep -q "perception" || exit 1'

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
