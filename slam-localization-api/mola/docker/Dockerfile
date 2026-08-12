FROM ros:humble-ros-base-jammy AS builder

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-colcon-common-extensions python3-rosdep build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/mola_ws/src
RUN git clone https://github.com/jlblancoc/nanoflann.git && \
    git clone https://github.com/MRPT/mrpt_ros.git && \
    git clone https://github.com/MOLAorg/mola_common.git && \
    git clone --recursive https://github.com/MOLAorg/mp2p_icp.git && \
    git clone --recursive https://github.com/MOLAorg/mola.git && \
    git clone https://github.com/MOLAorg/mola_state_estimation.git && \
    git clone https://github.com/MOLAorg/mola_imu_preintegration.git && \
    git clone https://github.com/MOLAorg/mola_sm_loop_closure.git && \
    git clone --recursive https://github.com/MOLAorg/mola_lidar_odometry.git
COPY mola_mid360_tools /opt/mola_ws/src/mola_mid360_tools

WORKDIR /opt/mola_ws
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble && \
    source /opt/ros/humble/setup.bash && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log

FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
COPY --from=builder /opt/mola_ws/src /opt/mola_ws/src
COPY --from=builder /opt/mola_ws/install /opt/mola_ws/install
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep ros-humble-rviz2 && \
    rosdep update && rosdep install --from-paths /opt/mola_ws/src --ignore-src -r -y --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*
COPY docker/entrypoint.sh /ros_entrypoint_mola.sh
RUN chmod +x /ros_entrypoint_mola.sh
ENTRYPOINT ["/ros_entrypoint_mola.sh"]
CMD ["bash"]
