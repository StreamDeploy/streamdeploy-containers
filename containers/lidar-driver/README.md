# LIDAR Driver

**Tagline:** LIDAR Driver + Visualizer (Velodyne, Ouster, RPLidar)

**Primary hardware:** Generic LIDARs via Ethernet/USB (arm64/amd64)

## What it does
Includes common LIDAR drivers and publishes ROS 2 topics + optional RViz snapshot service.

## Why it saves time
Fleet teams don’t want to rebuild for every sensor vendor; this gives a one‑line deployment baseline.

## Architectures
arm64, amd64

## Tags
ROS 2, LIDAR, Sensors

### Runtime notes

- Set `LIDAR_VENDOR` to `velodyne`, `rplidar` or `ouster` to select the appropriate driver.
- When using RPLidar set `DEVICE_PORT` (e.g. `/dev/ttyUSB0`) and `FRAME_ID`.
- Enable `RVIZ=true` to start rviz2 locally and `SNAPSHOT_SERVER=true` to expose a simple health endpoint on port 8081.