# MQTT Bridge

**Tagline:** ROS 2 ↔ MQTT Bridge (IoT/Cloud Integration)

**Primary hardware:** Generic Edge (arm64/amd64)

## What it does
Bridges ROS 2 topics to MQTT (with TLS) for external dashboards, IoT brokers or cloud services.

## Why it saves time
Every robotics company eventually hacks this together for monitoring — we give it as a clean primitive.

## Architectures
arm64, amd64

## Tags
ROS 2, MQTT, IoT, Cloud

### Runtime notes

- Specify the MQTT broker host and port via `MQTT_HOST` and `MQTT_PORT`.
- Use `ROS_SUBSCRIBE` and `ROS_PUBLISH` to map ROS 2 topics into MQTT topics and vice versa.
- Enable TLS by setting `MQTT_TLS=true` and mounting appropriate certificates.