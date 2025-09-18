# SLAM Navigation

**Tagline:** ROS 2 SLAM & Navigation Stack (Orin/RK3588‑first)

**Primary hardware:** NVIDIA Orin/Jetson, RK3588 (arm64)

## What it does
Prebuilt slam_toolbox + nav2 container with tuned GMapping/Cartographer backends, publishes maps and path plans.

## Why it saves time
Avoids multi‑hour ROS 2 build hell; provides a drop‑in navigation baseline for AMRs and mobile robots.

## Architectures
arm64

## Tags
ROS 2, SLAM, Navigation, Robotics

### Runtime notes

- Select between `slam_toolbox` and `cartographer` via `SLAM_STACK` environment variable.
- Set `NAV_STACK` to `nav2` (default) to run the Nav2 stack; other values will simply idle.
- Map, odometry and scan topics can be customised via environment variables.