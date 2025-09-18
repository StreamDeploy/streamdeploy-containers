#!/usr/bin/env sh
set -e
cat > mediamtx.yml <<EOF
rtspAddress: :${MTX_RTSP_PORT}
rtmpAddress: :${MTX_RTMP_PORT}
webrtc: ${MTX_WEBRTC}
paths:
  all:
    publishUser: "${MTX_AUTH_USER}"
    publishPass: "${MTX_AUTH_PASS}"
    readUser: "${MTX_AUTH_USER}"
    readPass: "${MTX_AUTH_PASS}"
EOF
exec mediamtx mediamtx.yml
