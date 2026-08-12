FROM ros:humble-ros-base-jammy

ARG TARGETARCH
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV PATH=/usr/local/cuda-12.6/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH}

# CUDA/GLIMのAPT repositoryはx86_64専用。arm64では別手順を使う。
RUN test "${TARGETARCH}" = "amd64" || \
    (echo >&2 "docker/all-in-one is amd64-only; see docker/ARM64_PLATFORMS.md"; exit 1)

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake curl git gpg python3-colcon-common-extensions python3-pip python3-rosdep \
    software-properties-common ros-humble-pcl-ros ros-humble-pcl-conversions \
    ros-humble-rviz2 ros-humble-tf2-eigen ros-humble-tf2-ros \
    && curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb -o /tmp/cuda-keyring.deb \
    && dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb \
    && curl -fsSL https://koide3.github.io/ppa/setup_ppa.sh | bash \
    && apt-get update && apt-get install -y --no-install-recommends \
       cuda-toolkit-12-6 libboost-all-dev libglfw3-dev libiridescence-dev \
       libmetis-dev libgtsam-points-cuda12.6-dev ros-humble-glim-ros-cuda12.6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git \
    && cmake -S Livox-SDK2 -B Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build Livox-SDK2/build -j"$(nproc)" \
    && cmake --install Livox-SDK2/build && rm -rf Livox-SDK2

WORKDIR /opt/livox_ws/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git \
    && source /opt/ros/humble/setup.bash \
    && cd livox_ros_driver2 && ./build.sh humble \
    && mkdir -p /opt/stack_defaults \
    && cp config/MID360_config.json /opt/stack_defaults/MID360_config.json \
    && rm -rf /opt/livox_ws/build /opt/livox_ws/log

WORKDIR /opt/localization_ws/src
RUN git clone --depth 1 https://github.com/rsasaki0109/ndt_omp_ros2.git \
    && git clone --depth 1 https://github.com/rsasaki0109/lidar_localization_ros2.git
COPY fix/apply_ndt_omp_target_link_fix.py /tmp/apply_ndt_omp_target_link_fix.py
WORKDIR /opt/localization_ws/src/lidar_localization_ros2
RUN python3 /tmp/apply_ndt_omp_target_link_fix.py CMakeLists.txt \
    && rm /tmp/apply_ndt_omp_target_link_fix.py
WORKDIR /opt/localization_ws
RUN source /opt/ros/humble/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
       --skip-keys "python3-pil ros_testing" \
    && python3 -m pip install --no-cache-dir Pillow \
    && python3 -c "from PIL import Image; print('Pillow import check: OK')"
RUN source /opt/ros/humble/setup.bash \
    && colcon build --merge-install --event-handlers console_direct+ \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log

COPY mid360_handheld.yaml mid360_handheld.rviz mid360_handheld_localization.launch.py /opt/guide/
COPY tools/mid360_mapping_filter.py /opt/guide/tools/mid360_mapping_filter.py
COPY docker/all-in-one/entrypoint.sh /ros_entrypoint_all_in_one.sh
COPY docker/all-in-one/run-all.sh /opt/stack/run-all.sh
RUN chmod +x /ros_entrypoint_all_in_one.sh /opt/stack/run-all.sh \
    && mkdir -p /data/config/glim /data/bags /data/maps
ENTRYPOINT ["/ros_entrypoint_all_in_one.sh"]
CMD ["/opt/stack/run-all.sh"]
