#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash

: "${USE_NAV2:=true}"
: "${MAP_FILE:=/maps/map.yaml}"
: "${CMD_VEL_TOPIC:=/cmd_vel}"
: "${UNITREE_GAIT_TOPIC:=/unitree/gait_cmd}"
: "${USE_SIM_TIME:=false}"

# Tiny Twist -> "gait command" bridge (placeholder topic)
python3 - << 'PY' &
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

cmd_vel_topic = os.getenv("CMD_VEL_TOPIC","/cmd_vel")
gait_topic = os.getenv("UNITREE_GAIT_TOPIC","/unitree/gait_cmd")

class Bridge(Node):
    def __init__(self):
        super().__init__('twist_to_gait')
        self.pub = self.create_publisher(String, gait_topic, 10)
        self.sub = self.create_subscription(Twist, cmd_vel_topic, self.on_cmd, 10)
    def on_cmd(self, msg: Twist):
        payload = {"vx": msg.linear.x, "vy": msg.linear.y, "wz": msg.angular.z}
        out = String(); out.data = str(payload)
        self.pub.publish(out)

rclpy.init(); rclpy.spin(Bridge())
PY

if [[ "${USE_NAV2}" == "true" ]]; then
  echo "[unitree-navigation] Starting Nav2 bringup"
  exec ros2 launch nav2_bringup bringup_launch.py use_sim_time:="${USE_SIM_TIME}" map:="${MAP_FILE}"
else
  echo "[unitree-navigation] Nav2 disabled; bridge running only"
  exec bash -lc "sleep infinity"
fi
