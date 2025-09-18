# Unitree Navigation Control

**Tagline:** Navigation & Control (Nav2 optional) + gait bridge

**Primary hardware:** Unitree H1 / G1 control PC (amd64)

## What it does
Runs Nav2 (optional) and a bridge that converts `/cmd_vel` into Unitree gait commands or a control topic.

## Why it saves time
Pre‑wired control stack with ROS 2 defaults and a simple Twist→gait bridge for quick bringup.

## Architectures
amd64

## Tags
ROS 2, Unitree, Nav2, Control, Humanoid

### Runtime notes

- Set `USE_NAV2=true` (default) to start the Nav2 bringup; set `false` to disable navigation and run only the gait bridge.
- `CMD_VEL_TOPIC` and `UNITREE_GAIT_TOPIC` can be overridden to match your control topics.
- Provide a map file via `MAP_FILE` when running Nav2; set `USE_SIM_TIME=true` when running under simulation.