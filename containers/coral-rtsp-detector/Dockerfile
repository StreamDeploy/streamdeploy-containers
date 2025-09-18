ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"debian:bookworm-slim"}

ENV DEBIAN_FRONTEND=noninteractive LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl ca-certificates python3 python3-pip python3-opencv \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-libav v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Edge TPU runtime + TFLite (CPU fallback if no TPU present)
RUN pip3 install --no-cache-dir \
    tflite-runtime==2.14.0 \
    numpy==1.26.4 pillow==10.3.0

# Install libedgetpu (USB accelerator)
RUN echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" \
    | tee /etc/apt/sources.list.d/coral-edgetpu.list && \
    curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add - && \
    apt-get update && apt-get install -y --no-install-recommends libedgetpu1-std && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

ENV MODEL_PATH=/models/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite \
    LABELS_PATH=/models/coco_labels.txt \
    CAMERA_DEV=/dev/video0 \
    RTSP_PORT=8554 \
    FPS=30 WIDTH=1280 HEIGHT=720

EXPOSE 8554
HEALTHCHECK --interval=30s --timeout=5s --start-period=20s \
  CMD bash -lc "nc -z localhost ${RTSP_PORT}"

ENTRYPOINT ["/app/entrypoint.sh"]
