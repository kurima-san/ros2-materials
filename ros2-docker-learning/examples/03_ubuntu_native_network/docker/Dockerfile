ARG ROS_DISTRO=lyrical
FROM ros:${ROS_DISTRO}-ros-base AS builder
SHELL ["/bin/bash", "-c"]
ARG ROS_DISTRO
WORKDIR /workspace
RUN apt-get update && apt-get install -y --no-install-recommends python3-colcon-common-extensions ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-tf2-ros && rm -rf /var/lib/apt/lists/*
COPY src ./src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --merge-install

FROM ros:${ROS_DISTRO}-ros-base AS runtime
SHELL ["/bin/bash", "-c"]
ARG ROS_DISTRO
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-tf2-ros && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
COPY --from=builder /workspace/install ./install
COPY docker/entrypoint.sh /usr/local/bin/ros-entrypoint.sh
RUN chmod +x /usr/local/bin/ros-entrypoint.sh
ENTRYPOINT ["/usr/local/bin/ros-entrypoint.sh"]
CMD ["ros2", "launch", "docker_ros2_lab", "lab.launch.py"]
