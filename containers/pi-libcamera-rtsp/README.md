# Pi Libcamera RTSP

**Tagline:** Raspberry Pi Camera → RTSP/HLS with libcamera + GStreamer

**Primary hardware:** Raspberry Pi 4/5 & CM4/CM5 (arm64)

## What it does
Captures from libcamera (Pi Cam v2/HQ/GS), encodes H.264/H.265 via V4L2 and serves RTSP (optional HLS).

## Why it saves time
No more libcamera/GStreamer incantations; healthcheck, env‑config and multi‑cam ready out of the box.

## Architectures
arm64

## Tags
Raspberry Pi, Camera, RTSP, HLS, GStreamer, arm64

### Runtime notes

- Mount the Pi camera device (usually `/dev/video0`) or specify `CAMERA_INDEX` for multiple cameras.
- Choose `CODEC=h265` to enable H.265 encoding on Raspberry Pi 5.
- This container uses `gst-rtsp-server` to expose the stream; HLS support can be added via `EXTRA_PIPELINE`.