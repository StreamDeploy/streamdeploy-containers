#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash

: "${NAV_STACK:=nav2}"
: "${SLAM_STACK:=slam_toolbox}"
: "${USE_SIM_TIME:=false}"

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros}

if [[ "${SLAM_STACK}" == "cartographer" ]]; then
  SLAM_CMD="ros2 launch cartographer_ros cartographer.launch.py use_sim_time:=${USE_SIM_TIME}"
else
  SLAM_CMD="ros2 launch slam_toolbox online_async_launch.py use_sim_time:=${USE_SIM_TIME}"
fi

if [[ "${NAV_STACK}" == "nav2" ]]; then
  NAV_CMD="ros2 launch nav2_bringup bringup_launch.py use_sim_time:=${USE_SIM_TIME}"
else
  NAV_CMD="bash -lc 'echo Unknown NAV_STACK=${NAV_STACK}; sleep infinity'"
fi

# Run both
bash -lc "${SLAM_CMD}" &
exec bash -lc "${NAV_CMD}"
