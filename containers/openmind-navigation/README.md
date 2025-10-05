# OpenMind Navigation

**Tagline:** ROS 2 navigation stack for autonomous robots

**Primary hardware:** NVIDIA Orin, Jetson Xavier (arm64)

## What it does
Provides modular navigation and path planning for autonomous robots with global/local path planning, obstacle avoidance, and goal execution. Integrates with ROS 2 and supports various locomotion configurations.

## Why it saves time
Drop-in navigation solution eliminates weeks of ROS 2 navigation tuning and configuration. Optimized for NVIDIA edge platforms with pre-configured path planners.

## Architectures
arm64

## Tags
ROS 2, Navigation, Path Planning, Robotics, Autonomous

### Runtime notes

- Configure ROS domain with `ROS_DOMAIN_ID` (default: 0)
- Set middleware implementation via `RMW_IMPLEMENTATION` (default: rmw_cyclonedx_cpp)
- Customize planner with `PLANNER_PLUGIN` (default: nav2_navfn_planner/NavfnPlanner)
- Customize controller with `CONTROLLER_PLUGIN` (default: nav2_dwb_controller/DWBLocalPlanner)
- Enable simulation time with `USE_SIM_TIME=true` for Gazebo/simulation environments
- Container exposes port 11511 for ROS 2 communication
- Requires map data and robot URDF for full functionality
- Mount custom configurations to `/opt/openmind/config/`
- Mount map files to `/opt/openmind/maps/`
