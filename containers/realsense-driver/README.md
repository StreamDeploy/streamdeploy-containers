# RealSense Driver

**Tagline:** Intel RealSense Camera ROS 2 Driver

**Primary hardware:** Intel RealSense D4xx/D435/D455, L515 (arm64/amd64)

## What it does
Sets up librealsense + ROS 2 wrapper, streams depth + RGB topics.

## Why it saves time
Skips kernel patching, udev rules and ROS driver compilation; instant depth camera integration.

## Architectures
arm64, amd64

## Tags
ROS 2, Depth Camera, Intel RealSense

### Runtime notes

- If you have multiple cameras, set the `SERIAL` environment variable to select the correct device.
- Sync depth and color streams by setting `ENABLE_SYNC=true`.
- Adjust frame resolutions and frame rates via the environment variables.