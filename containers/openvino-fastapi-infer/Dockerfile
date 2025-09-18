ARG BASE_IMAGE
FROM ${BASE_IMAGE:-"openvino/ubuntu22_runtime:2024.2.0"}
ENV DEBIAN_FRONTEND=noninteractive LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv curl ca-certificates && \
    rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir fastapi==0.111.0 uvicorn[standard]==0.30.0 pillow==10.3.0 numpy==1.26.4
WORKDIR /app
COPY entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh
ENV MODEL_XML=/models/model.xml \
    SERVICE_HOST=0.0.0.0 SERVICE_PORT=8080 \
    OV_DEVICE="AUTO"
EXPOSE 8080
HEALTHCHECK --interval=30s --timeout=5s --start-period=20s \
  CMD curl -fsS http://localhost:8080/health || exit 1
ENTRYPOINT ["/app/entrypoint.sh"]
