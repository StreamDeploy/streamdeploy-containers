# Vector Log Forwarder

**Tagline:** Tiny log/metric shipper for fleets (HTTP/MQTT/Syslog → S3/HTTP)

**Primary hardware:** Generic Edge (arm64/amd64)

## What it does
Uses Vector to tail container logs, parse JSON and forward to your backend or cloud object storage.

## Why it saves time
Batteries‑included log pipeline with structured parsing and backpressure—no custom agents needed.

## Architectures
arm64, amd64

## Tags
Logging, Metrics, Vector, Observability

### Runtime notes

- Set `SINK_URL` to point at your log ingestion endpoint (HTTP).  Without it, logs are only parsed and dropped.
- The healthcheck ensures the Vector HTTP server (port 8686) is up.
- Vector tails all Docker container logs; adjust the source configuration by editing `vector.yaml` if necessary.