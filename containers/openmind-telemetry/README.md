# OpenMind Telemetry

**Tagline:** Fleet monitoring and metrics collection

**Primary hardware:** NVIDIA Orin, Jetson Xavier (arm64)

## What it does
Telemetry and monitoring module for robotics fleets. Exposes health metrics, logs, and ROS topics with insights into system performance and operational status. Integrates with monitoring dashboards and observability platforms.

## Why it saves time
Ready-to-deploy monitoring eliminates weeks of custom telemetry development. Built-in integration with StreamDeploy dashboards and external observability tools like Prometheus and Grafana.

## Architectures
arm64

## Tags
ROS 2, Telemetry, Monitoring, Metrics, Robotics

### Runtime notes

- Configure ROS domain with `ROS_DOMAIN_ID` (default: 0)
- Set middleware implementation via `RMW_IMPLEMENTATION` (default: rmw_cyclonedx_cpp)
- Specify metrics API port with `METRICS_PORT` (default: 8080)
- Configure Prometheus port with `PROMETHEUS_PORT` (default: 9090)
- Set logging level with `LOG_LEVEL` (default: INFO)
- Enable system metrics with `COLLECT_SYSTEM_METRICS=true` (default: true)
- Enable ROS metrics with `COLLECT_ROS_METRICS=true` (default: true)
- Container exposes ports 8080 (REST API) and 9090 (Prometheus metrics)
- Health endpoint available at `/health` for monitoring
- Metrics endpoint available at `/metrics` for Prometheus scraping
- System status endpoint at `/status` for real-time monitoring
- Publishes ROS diagnostic messages to `/diagnostics` topic
- Automatically detects GPU metrics on NVIDIA hardware platforms
