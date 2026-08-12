FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV PATH=/usr/local/cuda-12.6/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH}

# GLIMのCUDA版とLivox ROS Driver 2を同じROS 2 Humble環境へ入れる。
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake curl git gpg python3-colcon-common-extensions \
    python3-rosdep software-properties-common wget \
    ros-humble-pcl-ros ros-humble-rviz2 \
    && curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
       -o /tmp/cuda-keyring.deb \
    && dpkg -i /tmp/cuda-keyring.deb \
    && rm /tmp/cuda-keyring.deb \
    && curl -fsSL https://koide3.github.io/ppa/setup_ppa.sh | bash \
    && apt-get update && apt-get install -y --no-install-recommends \
       cuda-toolkit-12-6 \
       libboost-all-dev libglfw3-dev libiridescence-dev libmetis-dev \
       libgtsam-points-cuda12.6-dev ros-humble-glim-ros-cuda12.6 \
    && rm -rf /var/lib/apt/lists/*

# MID-360用SDKとROS 2ドライバ。build.sh humbleは専用workspace内だけで実行する。
WORKDIR /tmp
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git \
    && cmake -S Livox-SDK2 -B Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build Livox-SDK2/build -j"$(nproc)" \
    && cmake --install Livox-SDK2/build \
    && rm -rf Livox-SDK2

WORKDIR /opt/livox_ws/src
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git \
    && source /opt/ros/humble/setup.bash \
    && cd livox_ros_driver2 \
    && ./build.sh humble \
    && mkdir -p /opt/mid360_defaults \
    && cp config/MID360_config.json /opt/mid360_defaults/MID360_config.json \
    && rm -rf /opt/livox_ws/build /opt/livox_ws/log

COPY tools/mid360_mapping_filter.py /opt/guide/tools/mid360_mapping_filter.py
COPY docker/glim-mid360/entrypoint.sh /ros_entrypoint_glim_mid360.sh
RUN chmod +x /ros_entrypoint_glim_mid360.sh \
    && mkdir -p /data/config /data/bags /data/maps

ENTRYPOINT ["/ros_entrypoint_glim_mid360.sh"]
CMD ["bash"]
