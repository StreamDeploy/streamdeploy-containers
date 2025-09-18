# Coral RTSP Detector

**Tagline:** Coral Edge TPU RTSP detector (Coral Dev Board/USB)

**Primary hardware:** Coral Dev Board / USB Accelerator (arm64/amd64 host)

## What it does
Runs TFLite Edge TPU detection and serves annotated RTSP stream.

## Why it saves time
Bypasses libedgetpu/tflite install pains; ready RTSP overlay and health endpoints.

## Architectures
arm64, amd64

## Tags
Coral, EdgeTPU, RTSP, TFLite

### Runtime notes

- Mount your video device (e.g. `/dev/video0`) and Edge TPU device (`/dev/bus/usb`) when running on a host.
- The container uses a Python script to draw bounding boxes and starts a separate GStreamer pipeline to serve RTSP.
- You can replace the model and labels by mounting custom files and overriding `MODEL_PATH` and `LABELS_PATH`.