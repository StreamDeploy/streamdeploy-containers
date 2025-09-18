# Edge Logger

**Tagline:** Unified Telemetry & Log Forwarder (Fluent Bit + gRPC)

**Primary hardware:** Generic Edge (arm64/amd64)

## What it does
Collects ROS 2 logs, system metrics (CPU/GPU temp, disk, network) and ships them to central logging via gRPC/HTTP.

## Why it saves time
Debugging fleets is a nightmare without centralized telemetry — this container makes it turn‑key.

## Architectures
arm64, amd64

## Tags
Logging, Telemetry, Observability

### Runtime notes

- Set `SINK_URL` to your log/metric collector endpoint (HTTP) to forward logs; if unset the data is only exposed via the built‑in HTTP server on port 2020.
- Use `HOSTNAME` and `EXTRA_LABELS` to tag your metrics and logs.
- The container uses Fluent Bit to collect system and ROS 2 service metrics.