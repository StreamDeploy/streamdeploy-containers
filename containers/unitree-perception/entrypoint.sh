#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash

: "${CAMERA_TOPIC:=/camera/image_raw}"
: "${DETECTION_TOPIC:=/perception/detections}"
: "${MODEL_PATH:=/models/yolov8n.onnx}"
: "${CONF_THRESHOLD:=0.35}"

python3 - << 'PY'
import os, io, time
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

model = os.getenv("MODEL_PATH","/models/yolov8n.onnx")
conf = float(os.getenv("CONF_THRESHOLD","0.35"))
providers = ["CUDAExecutionProvider","CPUExecutionProvider"]
try:
    session = ort.InferenceSession(model, providers=providers)
except Exception:
    session = ort.InferenceSession(model, providers=["CPUExecutionProvider"])

class Perception(Node):
    def __init__(self):
        super().__init__('unitree_perception')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(String, os.getenv("DETECTION_TOPIC","/perception/detections"), 10)
        self.sub = self.create_subscription(Image, os.getenv("CAMERA_TOPIC","/camera/image_raw"), self.on_img, 10)
        self.iname = session.get_inputs()[0].name
    def on_img(self, msg: Image):
        try:
            cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            from PIL import Image as PImage
            im = PImage.fromarray(cv[..., ::-1]).resize((640,640))
            x = np.asarray(im).astype(np.float32)/255.0
            x = np.transpose(x, (2,0,1))[None, ...]
            outs = session.run(None, {self.iname: x})
            payload = {"outputs": [np.array(o).shape for o in outs], "conf_threshold": conf}
            s = String(); s.data = str(payload)
            self.pub.publish(s)
        except Exception as e:
            self.get_logger().warn(f"infer error: {e}")
rclpy.init(); rclpy.spin(Perception())
PY
