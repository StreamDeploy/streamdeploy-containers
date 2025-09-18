# YOLOv8 Detector

**Tagline:** YOLOv8 Detector with ONNX/TensorRT Acceleration

**Primary hardware:** NVIDIA Orin/Jetson, RK3588 (arm64)

## What it does
Loads Ultralytics YOLOv8 ONNX weights, runs GPU‑accelerated inference, publishes detections via REST or ROS 2 topic.

## Why it saves time
Standard model every robotics dev asks for — tuned builds for Jetson & RK3588, no dependency pain.

## Architectures
arm64

## Tags
ONNX, TensorRT, Computer Vision, YOLO

### Runtime notes

- Set `MODE=rest` (default) to expose a FastAPI server on port 8080; use `/health` and `/infer` endpoints.
- Set `MODE=ros2` to publish dummy detection messages on the `ROS_TOPIC` instead (customise for your camera input).
- Mount your YOLOv8 ONNX file under `/models/yolov8.onnx` or override `MODEL_PATH`.