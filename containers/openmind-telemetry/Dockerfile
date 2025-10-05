ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"ros:humble-ros-base"}
USER root

# Install telemetry dependencies and tools
RUN apt-get update && apt-get install -y \
    bash \
    ca-certificates \
    curl \
    netcat-traditional \
    python3-pip \
    python3-yaml \
    python3-psutil \
    ros-humble-diagnostic-msgs \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Python telemetry libraries
RUN pip3 install --no-cache-dir \
    flask \
    prometheus_client \
    psutil \
    pynvml \
    requests

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Environment variables for configuration
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedx_cpp \
    METRICS_PORT=8080 \
    PROMETHEUS_PORT=9090 \
    LOG_LEVEL="INFO" \
    COLLECT_SYSTEM_METRICS=true \
    COLLECT_ROS_METRICS=true

# Create telemetry directory
RUN mkdir -p /opt/openmind/telemetry

# Expose metrics ports
EXPOSE 8080 9090

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s \
    CMD curl -f http://localhost:8080/health || exit 1

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
