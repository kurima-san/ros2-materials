FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-colcon-common-extensions python3-rosdep \
    ros-humble-pcl-ros ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git \
    && cmake -S Livox-SDK2 -B Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build Livox-SDK2/build -j"$(nproc)" \
    && cmake --install Livox-SDK2/build \
    && rm -rf Livox-SDK2

WORKDIR /opt/livox_ws/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git \
    && source /opt/ros/humble/setup.bash \
    && cd livox_ros_driver2 && ./build.sh humble \
    && mkdir -p /opt/mid360_defaults \
    && cp config/MID360_config.json /opt/mid360_defaults/MID360_config.json \
    && rm -rf /opt/livox_ws/build /opt/livox_ws/log

COPY docker/mid360/entrypoint.sh /ros_entrypoint_mid360.sh
RUN chmod +x /ros_entrypoint_mid360.sh && mkdir -p /data/config /data/bags
ENTRYPOINT ["/ros_entrypoint_mid360.sh"]
CMD ["bash"]
