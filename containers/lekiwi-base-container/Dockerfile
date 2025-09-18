ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"python:3.11-slim-bookworm"}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    PYTHONUNBUFFERED=1

# System dependencies for LeKiwi
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential curl ca-certificates git \
    libopencv-dev python3-opencv \
    procps tini sudo \
    && rm -rf /var/lib/apt/lists/*

# Create robot user (non-root)
RUN useradd -ms /bin/bash robot && \
    echo "robot ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER robot
WORKDIR /home/robot

# Install LeKiwi dependencies
RUN pip3 install --no-cache-dir --user \
    lerobot[lekiwi]==0.2.0 \
    opencv-python-headless==4.9.0.80 \
    numpy==1.26.4

# Copy LeKiwi configuration and scripts
COPY --chown=robot:robot lekiwi_config.yaml /home/robot/lekiwi_config.yaml
COPY --chown=robot:robot entrypoint.sh /home/robot/entrypoint.sh
RUN chmod +x /home/robot/entrypoint.sh

# Environment defaults
ENV ROBOT_ID=my-kiwi \
    DEPLOY_ENV=production \
    LOG_LEVEL=INFO \
    CONFIG_PATH=/home/robot/lekiwi_config.yaml

# Health check for LeKiwi host agent
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s \
  CMD pgrep -f "lerobot.robots.lekiwi.lekiwi_host" || exit 1

# Use tini for proper signal handling
ENTRYPOINT ["/usr/bin/tini", "--", "/home/robot/entrypoint.sh"]