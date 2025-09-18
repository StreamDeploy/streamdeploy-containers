# ROS 2 Vision Streamer

**Tagline:** Multi-camera ROS 2 + GStreamer (Jetson/Orin-first)

**Primary hardware:** NVIDIA Orin/Jetson (arm64)

## What it does
Multi-camera capture, H.264/H.265 encode via GStreamer; publishes ROS 2 topics; optional RTSP.

## Why it saves time
Skip driver wrangling, NVENC plumbing, ROS 2 packages and camera calibration boilerplate.

## Architectures
arm64

## Tags
ROS 2, GStreamer, RTSP, NVENC, Jetson, arm64

### Runtime notes

- Set `ENABLE_RTSP=true` to expose an RTSP stream on `$RTSP_PORT` (default 8554).
- Mount `/dev/video*` devices as needed.
- Adjust encoding and framerate via environment variables (`ENCODER`, `FPS`, `WIDTH`, `HEIGHT`).