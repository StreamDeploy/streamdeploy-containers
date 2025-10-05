# OpenMind Perception

**Tagline:** Real-time perception for robotics edge AI

**Primary hardware:** NVIDIA Orin, Jetson Xavier (arm64)

## What it does
Real-time perception module designed for robots and edge AI devices. Provides vision, object detection, and tracking capabilities with multimodal sensing integration. Optimized for hardware acceleration on NVIDIA platforms.

## Why it saves time
Pre-built perception pipeline eliminates months of computer vision development. Hardware-optimized inference with ROS 2 integration and ready-to-use object detection models.

## Architectures
arm64

## Tags
ROS 2, Perception, Computer Vision, Object Detection, Robotics

### Runtime notes

- Configure ROS domain with `ROS_DOMAIN_ID` (default: 0)
- Set middleware implementation via `RMW_IMPLEMENTATION` (default: rmw_cyclonedx_cpp)
- Specify model directory with `MODEL_PATH` (default: /opt/openmind/models)
- Choose detection model with `DETECTION_MODEL` (default: yolov8n.pt)
- Set input camera topic via `INPUT_TOPIC` (default: /camera/image_raw)
- Set output detections topic via `OUTPUT_TOPIC` (default: /perception/detections)
- Adjust confidence threshold with `CONFIDENCE_THRESHOLD` (default: 0.5)
- Container automatically downloads YOLOv8 models on first run
- Publishes Detection2DArray messages compatible with ROS 2 vision stack
- Mount custom models to `/opt/openmind/models/` directory
- Supports GPU acceleration on NVIDIA hardware platforms
