# Triton Robot Inference

**Tagline:** TensorRT/ONNX runtime FastAPI microservice (Orin-first)

**Primary hardware:** NVIDIA Orin/Jetson (arm64)

## What it does
Lightweight FastAPI microservice running ONNX Runtime/TensorRT for common vision models.

## Why it saves time
One-command GPU inference server; avoids building TRT engines by hand or stitching CUDA libs.

## Architectures
arm64

## Tags
FastAPI, ONNX, TensorRT, Jetson, REST

### Runtime notes

- Mount your ONNX model under `/models/model.onnx` or set `MODEL_PATH` accordingly.
- The service exposes `/health` and `/infer` endpoints on port 8080.
- Provide image files via multipart/form‑data to `/infer` to perform inference.