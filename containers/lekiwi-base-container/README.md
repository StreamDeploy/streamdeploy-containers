# LeKiwi Base Container

**Tagline:** LeKiwi Robot Base Container (Raspberry Pi optimized)

**Primary hardware:** Raspberry Pi 4/5 & CM4/CM5 (arm64)

## What it does
Complete LeKiwi robot software stack with host agent, configuration management, and Pi‑optimized runtime.

## Why it saves time
Zero‑setup LeKiwi deployment with pre‑configured environment, health monitoring, and Pi‑specific optimizations.

## Architectures
arm64

## Tags
LeKiwi, Raspberry Pi, Robot, Host Agent, arm64

### Runtime notes

- Mount any required devices (e.g. `/dev/video*` if using cameras).
- Populate `lekiwi_config.yaml` with your robot configuration and mount into the container at runtime.
- Adjust environment variables as needed: `ROBOT_ID`, `DEPLOY_ENV`, `LOG_LEVEL`.
- The container uses a non‑root `robot` user for improved security.