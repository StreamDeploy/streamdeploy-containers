#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash

case "${LIDAR_VENDOR}" in
  velodyne)
    CMD="ros2 launch velodyne_pointcloud VLP16-32c-ros2.launch.py"
    ;;
  rplidar)
    CMD="ros2 launch rplidar_ros rplidar.launch.py serial_port:=${DEVICE_PORT} frame_id:=${FRAME_ID}"
    ;;
  ouster)
    # Expect an external ouster driver (user can mount), fallback to velodyne warning
    echo "[warn] Ouster not bundled via apt; please mount driver. Attempting velodyne config as placeholder."
    CMD="bash -lc 'sleep infinity'"
    ;;
  *)
    echo "[err] Unsupported LIDAR_VENDOR=${LIDAR_VENDOR}"
    exit 1
    ;;
esac

if [[ "${RVIZ}" == "true" ]]; then
  (rviz2 -d '' 2>/dev/null || true) &
fi

if [[ "${SNAPSHOT_SERVER}" == "true" ]]; then
  python3 - <<'PY' &
from flask import Flask, jsonify
app = Flask(__name__)
@app.get("/health")
def h(): return jsonify(ok=True)
app.run(host="0.0.0.0", port=8081)
PY
fi

exec bash -lc "${CMD}"
