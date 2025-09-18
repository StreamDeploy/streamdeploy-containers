#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/${ROS_DISTRO}/setup.bash

ARGS=()
[[ -n "${SERIAL}" ]] && ARGS+=( "serial_no:=${SERIAL}" )
ARGS+=( "enable_sync:=${ENABLE_SYNC}" )
ARGS+=( "color_width:=${COLOR_WIDTH}" "color_height:=${COLOR_HEIGHT}" "color_fps:=${COLOR_FPS}" )
ARGS+=( "depth_width:=${DEPTH_WIDTH}" "depth_height:=${DEPTH_HEIGHT}" "depth_fps:=${DEPTH_FPS}" )

exec ros2 launch realsense2_camera rs_launch.py "${ARGS[@]}"
