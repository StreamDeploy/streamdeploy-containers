ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"ubuntu:22.04"}
ENV DEBIAN_FRONTEND=noninteractive LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Basic deps + GStreamer + MPP userspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-libav v4l-utils curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Expect host to provide /dev/video* and /dev/mpp_service; pass with --device
WORKDIR /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

ENV CAMERA_DEV=/dev/video0 \
    WIDTH=1280 HEIGHT=720 FPS=30 \
    CODEC=h264 \
    RTSP_PORT=8554 \
    EXTRA_PIPELINE=""

EXPOSE 8554
HEALTHCHECK --interval=30s --timeout=5s --start-period=20s \
  CMD bash -lc 'nc -z localhost $RTSP_PORT'

ENTRYPOINT ["/app/entrypoint.sh"]
