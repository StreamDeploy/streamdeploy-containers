#!/usr/bin/env bash
set -euo pipefail
: "${CAMERA_INDEX:=0}"
: "${WIDTH:=1280}"; : "${HEIGHT:=720}"; : "${FPS:=30}"
: "${CODEC:=h264}"; : "${RTSP_PORT:=8554}"; : "${RTSP_PATH:=/cam}"
: "${EXTRA_PIPELINE:=}"

# Select encoder caps for Pi (v4l2 stateful)
if [[ "${CODEC}" == "h265" ]]; then
  ENC="v4l2h265enc"
  RTP="rtph265pay pt=96"
else
  ENC="v4l2h264enc extra-controls=\"controls,video_gop_size=30\""
  RTP="rtph264pay pt=96 config-interval=1"
fi

# libcamera source via libcamerasrc (if present) else fallback to libcamera-vid piping
if gst-inspect-1.0 libcamerasrc >/dev/null 2>&1; then
  PIPE="libcamerasrc camera-index=${CAMERA_INDEX} ! video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1"
else
  echo "[warn] libcamerasrc not found; attempting v4l2src /dev/video0"
  PIPE="v4l2src device=/dev/video0 ! video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1"
fi

# Run RTSP server using gst-launch (simple, robust)
exec gst-launch-1.0 -v ${PIPE} ! videoconvert ! ${ENC} ! ${RTP} name=pay0 ${EXTRA_PIPELINE} \
  ! rtspclientsink location=rtsp://127.0.0.1:${RTSP_PORT}${RTSP_PATH} || true
