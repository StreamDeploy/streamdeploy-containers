#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash || true

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Optional overlay sourced if present
if [[ -d "/work/ros_ws/install" ]]; then
  echo "[unitree-base] Sourcing /work/ros_ws/install/setup.bash"
  source /work/ros_ws/install/setup.bash
fi

LAUNCH_PKG="${BRINGUP_PACKAGE:-unitree_bringup}"
LAUNCH_FILE="${BRINGUP_LAUNCH:-humanoid_bringup.launch.py}"

if ros2 pkg list | grep -q "${LAUNCH_PKG}"; then
  echo "[unitree-base] Launching ${LAUNCH_PKG} ${LAUNCH_FILE}"
  exec ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" robot_model:="${ROBOT_MODEL:-H1}" nic:="${NETWORK_INTERFACE:-eth0}"
else
  echo "[unitree-base] Package ${LAUNCH_PKG} not found. Starting heartbeat node."
  python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
class HB(Node):
    def __init__(self):
        super().__init__('unitree_base_heartbeat')
        self.create_timer(1.0, self.tick)
    def tick(self):
        self.get_logger().info('unitree_base heartbeat...')
rclpy.init(); rclpy.spin(HB())
PY
fi
