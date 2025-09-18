#!/usr/bin/env bash
set -euo pipefail
: "${CODEC:=h264}"
if [[ "${CODEC}" == "h265" ]]; then
  ENC="mpph265enc"
  RTP="rtph265pay pt=96"
else
  ENC="mpph264enc"
  RTP="rtph264pay pt=96 config-interval=1"
fi

exec gst-launch-1.0 -v v4l2src device="${CAMERA_DEV}" ! \
  video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 ! \
  videoconvert ! ${ENC} ! ${RTP} name=pay0 ${EXTRA_PIPELINE} \
  ! rtspclientsink location=rtsp://127.0.0.1:${RTSP_PORT}/stream
