#!/usr/bin/env bash
set -euo pipefail

# Default environment variables
: "${ROS_DOMAIN_ID:=0}"
: "${RMW_IMPLEMENTATION:=rmw_cyclonedx_cpp}"
: "${PLANNER_PLUGIN:=nav2_navfn_planner/NavfnPlanner}"
: "${CONTROLLER_PLUGIN:=nav2_dwb_controller/DWBLocalPlanner}"
: "${USE_SIM_TIME:=false}"

# Create necessary directories
mkdir -p /opt/openmind/config /opt/openmind/maps

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

echo "Starting OpenMind Navigation with:"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "  PLANNER_PLUGIN: ${PLANNER_PLUGIN}"
echo "  CONTROLLER_PLUGIN: ${CONTROLLER_PLUGIN}"
echo "  USE_SIM_TIME: ${USE_SIM_TIME}"

# Generate navigation configuration
CONFIG_FILE="/opt/openmind/config/nav2_params.yaml"
cat > "${CONFIG_FILE}" <<EOF
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: ${USE_SIM_TIME}
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "${PLANNER_PLUGIN}"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

controller_server:
  ros__parameters:
    use_sim_time: ${USE_SIM_TIME}
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "${CONTROLLER_PLUGIN}"

bt_navigator:
  ros__parameters:
    use_sim_time: ${USE_SIM_TIME}
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"

waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200
EOF

# Set environment variables
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION
export USE_SIM_TIME

# Launch the navigation stack
echo "Launching OpenMind Navigation Stack..."
exec ros2 launch nav2_bringup navigation_launch.py \
  params_file:"${CONFIG_FILE}" \
  use_sim_time:="${USE_SIM_TIME}"
