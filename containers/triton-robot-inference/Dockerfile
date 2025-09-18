ARG BASE_IMAGE
# Jetson-friendly CUDA/TensorRT runtime image
FROM ${BASE_IMAGE:-"nvcr.io/nvidia/l4t-pytorch:r35.4.1-py3"}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev build-essential curl ca-certificates && \
    rm -rf /var/lib/apt/lists/*

# ONNX Runtime (GPU for Jetson uses special builds; fallback to CPU if unavailable)
RUN pip3 install --no-cache-dir \
    fastapi==0.111.0 uvicorn[standard]==0.30.0 \
    numpy==1.26.4 pillow==10.3.0 onnx==1.16.0 onnxruntime-gpu==1.18.0

# Place model(s) (mount your own at runtime or bake here)
WORKDIR /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh
# Example: default ONNX path (override with MODEL_PATH env)
ENV MODEL_PATH=/models/model.onnx \
    SERVICE_HOST=0.0.0.0 \
    SERVICE_PORT=8080

EXPOSE 8080
HEALTHCHECK --interval=30s --timeout=5s --start-period=20s \
  CMD curl -fsS http://localhost:8080/health || exit 1

ENTRYPOINT ["/app/entrypoint.sh"]
