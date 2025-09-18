# MediaMTX Edge Relay

**Tagline:** Unified RTSP/RTMP/WebRTC ingest & restream for fleets

**Primary hardware:** Generic Edge (arm64/amd64)

## What it does
Runs MediaMTX (formerly rtsp-simple-server) to fan‑in cameras and restream to RTSP/RTMP/WebRTC.

## Why it saves time
One binary to terminate weird camera protocols, add auth and expose a consistent URL to your apps.

## Architectures
arm64, amd64

## Tags
MediaMTX, RTSP, RTMP, WebRTC, Gateway

### Runtime notes

- Set `MTX_AUTH_USER` and `MTX_AUTH_PASS` to enable basic authentication for publishing/reading streams.
- Ports 8554 (RTSP), 1935 (RTMP), 8889 and 8189 (WebRTC) are exposed.
- The server reads its configuration from `mediamtx.yml` generated at runtime.