FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-colcon-common-extensions python3-rosdep python3-pip \
    ros-humble-rviz2 ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/ros2_ws/src
RUN git clone https://github.com/rsasaki0109/lidar_localization_ros2.git
COPY fix/CMakeLists.txt /opt/ros2_ws/src/lidar_localization_ros2/CMakeLists.txt
COPY mid360_handheld.yaml mid360_handheld.rviz mid360_handheld_localization.launch.py \
     /opt/guide/

WORKDIR /opt/ros2_ws
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble && \
    source /opt/ros/humble/setup.bash && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log

COPY docker/entrypoint.sh /ros_entrypoint_localization.sh
RUN chmod +x /ros_entrypoint_localization.sh
ENTRYPOINT ["/ros_entrypoint_localization.sh"]
CMD ["bash"]
