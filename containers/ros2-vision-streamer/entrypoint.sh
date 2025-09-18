#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash

# Build a GStreamer pipeline depending on device
if [[ "${ENABLE_RTSP}" == "true" ]]; then
  # RTSP + ROS 2 publish (Jetson can use nvarguscamerasrc; fallback to v4l2src)
  PIPELINE="v4l2src device=${CAMERA_DEV} ! videoconvert ! video/x-raw,framerate=${FPS}/1,width=${WIDTH},height=${HEIGHT} ! x264enc tune=zerolatency bitrate=4000 speed-preset=ultrafast ! rtph264pay name=pay0 pt=96"
  # Simple RTSP server
  gst-rtsp-server-1.0 &>/dev/null || true
fi

# Basic ROS 2 image publisher using OpenCV (placeholder; swap with your node)
python3 - << 'PY'
import os, time, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

dev = os.environ.get("CAMERA_DEV","/dev/video0")
fps = int(os.environ.get("FPS","30"))

class CamNode(Node):
    def __init__(self):
        super().__init__('camera_pub')
        self.pub = self.create_publisher(Image, 'image_raw', 10)
        self.cap = cv2.VideoCapture(dev)
        self.br = CvBridge()
        timer_period = 1.0/max(1,fps)
        self.timer = self.create_timer(timer_period, self.tick)
    def tick(self):
        ok, frame = self.cap.read()
        if not ok: return
        msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)

rclpy.init()
node = CamNode()
rclpy.spin(node)
PY
