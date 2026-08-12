FROM ros:humble-ros-base-jammy

ARG TARGETARCH
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV PATH=/usr/local/cuda-12.6/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:${LD_LIBRARY_PATH}

# CUDA/GLIMのAPT repositoryはx86_64専用。arm64では別手順を使う。
RUN test "${TARGETARCH}" = "amd64" || \
    (echo >&2 "docker/glim is amd64-only; see docker/ARM64_PLATFORMS.md"; exit 1)

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gpg software-properties-common ros-humble-rviz2 \
    && curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb -o /tmp/cuda-keyring.deb \
    && dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb \
    && curl -fsSL https://koide3.github.io/ppa/setup_ppa.sh | bash \
    && apt-get update && apt-get install -y --no-install-recommends \
       cuda-toolkit-12-6 libboost-all-dev libglfw3-dev libiridescence-dev \
       libmetis-dev libgtsam-points-cuda12.6-dev ros-humble-glim-ros-cuda12.6 \
    && rm -rf /var/lib/apt/lists/*

COPY tools/mid360_mapping_filter.py /opt/guide/tools/mid360_mapping_filter.py
COPY docker/glim/entrypoint.sh /ros_entrypoint_glim.sh
RUN chmod +x /ros_entrypoint_glim.sh && mkdir -p /data/config/glim /data/bags /data/maps
ENTRYPOINT ["/ros_entrypoint_glim.sh"]
CMD ["bash"]
