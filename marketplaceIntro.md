Fast‑start containers for ROS 2 camera streaming, TensorRT inference and Coral Edge TPU detection.  Designed for NVIDIA Orin/Jetson and similar edge AI hardware.

All containers support the following conventions:

* `DEPLOY_ENV` – environment selection for your deployment
* Structured JSON logs on stdout/stderr
* Healthchecks baked into each container image
* Configurable camera/device sources via environment variables

Use with StreamDeploy’s deployment pipeline for robots to get OTA updates, versioned configs and one‑command rollbacks.