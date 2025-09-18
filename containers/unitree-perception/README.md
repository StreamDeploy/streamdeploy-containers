# Unitree Perception

**Tagline:** Perception Inference (ONNX/TensorRT) → ROS 2 detections

**Primary hardware:** Unitree control PC (amd64, optional NVIDIA GPU)

## What it does
Subscribes to camera topic, runs ONNX/TensorRT model and publishes detection results to ROS 2.

## Why it saves time
Drop‑in perception service with CPU/GPU fallback, ROS 2‑ready topics and healthchecks.

## Architectures
amd64

## Tags
ROS 2, Unitree, Perception, ONNX, Humanoid

### Runtime notes

- Set `CAMERA_TOPIC` and `DETECTION_TOPIC` to integrate with your camera and downstream consumers.
- Mount your ONNX model under `/models/yolov8n.onnx` or override `MODEL_PATH`.
- Tune `CONF_THRESHOLD` to adjust detection confidence filtering.