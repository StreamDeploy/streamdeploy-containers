#!/usr/bin/env bash
set -euo pipefail

if [[ "${MODE}" == "ros2" ]]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
  python3 - << 'PY'
import os, time, onnxruntime as ort, numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

model = os.getenv("MODEL_PATH","/models/yolov8.onnx")
session = ort.InferenceSession(model, providers=["CUDAExecutionProvider","CPUExecutionProvider"])

class Pub(Node):
    def __init__(self):
        super().__init__('yolo_pub')
        self.pub = self.create_publisher(String, os.getenv("ROS_TOPIC","/detections"), 10)
        self.timer = self.create_timer(1.0, self.tick)
    def tick(self):
        # Dummy payload; wire your camera/input
        out = {"boxes":[[0,0,10,10]],"classes":[0],"scores":[0.99]}
        msg = String()
        msg.data = str(out)
        self.pub.publish(msg)

rclpy.init(); node = Pub(); rclpy.spin(node)
PY
else
  cat > /app/server.py << 'PY'
import os, io
from fastapi import FastAPI, UploadFile, File
from fastapi.responses import JSONResponse
from PIL import Image
import numpy as np, onnxruntime as ort

app = FastAPI()
model_path = os.getenv("MODEL_PATH","/models/yolov8.onnx")
session = ort.InferenceSession(model_path, providers=["CUDAExecutionProvider","CPUExecutionProvider"])

@app.get("/health")
def health(): return {"ok": True, "model": os.path.basename(model_path), "providers": session.get_providers()}

@app.post("/infer")
async def infer(file: UploadFile = File(...)):
    img = Image.open(io.BytesIO(await file.read())).convert("RGB").resize((640,640))
    x = np.asarray(img).astype(np.float32)/255.0
    x = np.transpose(x, (2,0,1))[None, ...]
    outputs = session.run(None, {session.get_inputs()[0].name: x})
    return JSONResponse({"outputs":[o.tolist() for o in outputs]})
PY
  exec python3 -m uvicorn server:app --host "${SERVICE_HOST}" --port "${SERVICE_PORT}"
fi
