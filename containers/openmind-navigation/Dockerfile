ARG BASE_IMAGE
FROM ${BASE_IMAGE:-ros:humble-ros-base}
USER root

# Install navigation dependencies and tools
RUN apt-get update && apt-get install -y \
    bash \
    ca-certificates \
    netcat-traditional \
    ros-humble-nav2-bringup \
    ros-humble-nav2-core \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Environment variables for configuration
ENV ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedx_cpp \
    PLANNER_PLUGIN="nav2_navfn_planner/NavfnPlanner" \
    CONTROLLER_PLUGIN="nav2_dwb_controller/DWBLocalPlanner" \
    USE_SIM_TIME=false

# Expose common ROS 2 ports
EXPOSE 11511

HEALTHCHECK --interval=30s --timeout=10s --start-period=30s \
    CMD bash -lc 'source /opt/ros/humble/setup.bash && ros2 node list | grep -q "nav2" || exit 1'

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
