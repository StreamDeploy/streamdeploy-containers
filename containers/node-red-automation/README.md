# Node‑RED Automation

**Tagline:** Node‑RED flows for edge orchestration (MQTT/HTTP/ROS bridge)

**Primary hardware:** Generic Edge (arm64/amd64)

## What it does
Low‑code automations: bridge MQTT topics, HTTP webhooks and ROS endpoints for quick prototypes.

## Why it saves time
Ship a ready Node‑RED with curated nodes and healthchecks; mount flows and go.

## Architectures
arm64, amd64

## Tags
Node‑RED, Automation, MQTT, Bridge

### Runtime notes

- Bind mount your Node‑RED flows into `/data/flows.json` to persist or preseed your automation logic.
- The healthcheck polls the Node‑RED web UI on port 1880.
- Additional nodes can be installed by extending the Dockerfile.