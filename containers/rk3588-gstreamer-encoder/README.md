# RK3588 GStreamer Encoder

**Tagline:** RK3588 MPP/NPU-friendly H.264/H.265 hardware encoder + RTSP

**Primary hardware:** ROC‑RK3588 / Orange Pi 5 (arm64)

## What it does
Uses Rockchip MPP (rkmpp) via GStreamer to encode camera/video to RTSP with low CPU.

## Why it saves time
Avoids manual rkmpp plumbing and kernel driver quirks; one env‑config to stream reliably.

## Architectures
arm64

## Tags
RK3588, GStreamer, RTSP, rkmpp, arm64

### Runtime notes

- Requires access to `/dev/video*` and `/dev/mpp_service`; run the container with `--device` flags.
- Choose `CODEC=h265` to use the H.265 encoder; defaults to H.264.
- Additional GStreamer elements can be appended via `EXTRA_PIPELINE`.