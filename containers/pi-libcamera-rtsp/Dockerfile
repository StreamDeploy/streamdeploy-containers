ARG BASE_IMAGE
# Pi OS arm64 base with libcamera & gstreamer
FROM ${BASE_IMAGE:-"raspberrypi/libcamera:latest"}

ENV DEBIAN_FRONTEND=noninteractive LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-libav libcamera-apps \
    python3 python3-pip curl ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# Minimal rtsp server helper
RUN pip3 install --no-cache-dir gst-rtsp-server

WORKDIR /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

ENV CAMERA_INDEX=0 \
    WIDTH=1280 HEIGHT=720 FPS=30 \
    CODEC=h264 \
    RTSP_PORT=8554 \
    RTSP_PATH=/cam \
    EXTRA_PIPELINE=""

EXPOSE 8554
HEALTHCHECK --interval=30s --timeout=5s --start-period=20s \
  CMD bash -lc 'nc -z localhost $RTSP_PORT'

ENTRYPOINT ["/app/entrypoint.sh"]
