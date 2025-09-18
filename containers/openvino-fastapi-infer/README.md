# OpenVINO FastAPI Infer

**Tagline:** Intel OpenVINO inference microservice (CPU/iGPU/NPU)

**Primary hardware:** Intel NUC / Arc / Core Ultra (amd64)

## What it does
FastAPI wrapper around OpenVINO Runtime for image models, auto‑selecting the best device.

## Why it saves time
Skip SDK installs and sample refactors; drop a model and get `/health` and `/infer` immediately.

## Architectures
amd64

## Tags
OpenVINO, Intel, Inference, REST, amd64

### Runtime notes

- Mount your OpenVINO IR model files under `/models/model.xml` and `/models/model.bin` or override `MODEL_XML`.
- The container chooses the best available device automatically (CPU/GPU/NPU) when `OV_DEVICE=AUTO`.
- Exposes port 8080; use `/health` for readiness and `/infer` for inference.