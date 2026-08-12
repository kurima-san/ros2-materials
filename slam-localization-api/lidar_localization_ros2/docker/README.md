# lidar_localization_ros2 Docker 環境

Ubuntu 22.04 / ROS 2 Humble と、このガイドの修正版 `CMakeLists.txt` を再現可能なイメージにまとめます。

```bash
cd slam-localization-api/lidar_localization_ros2/docker
docker compose build
xhost +local:docker                 # RVizを使うLinuxホストだけ
docker compose run --rm localization
```

ホストの `~/ros2_maps` と `~/ros_bags` は、コンテナ内の `/data/maps` と `/data/bags` に対応します。MID-360を直接購読するため `network_mode: host` を使います。

## 別PCへオフライン配布

作成側（amd64で作ったイメージはamd64へ、arm64はarm64へ配布）:

```bash
docker image inspect lidar-localization-mid360:humble >/dev/null
docker save lidar-localization-mid360:humble | gzip > lidar-localization-mid360_humble.tar.gz
sha256sum lidar-localization-mid360_humble.tar.gz > lidar-localization-mid360_humble.tar.gz.sha256
```

`tar.gz`、`sha256`、この `compose.yaml` をUSBメモリ等で対象PCへコピーし、対象PCで:

```bash
sha256sum -c lidar-localization-mid360_humble.tar.gz.sha256
gunzip -c lidar-localization-mid360_humble.tar.gz | docker load
docker compose -f compose.yaml run --rm localization
```

レジストリを使う場合は、配布先名へ `docker tag` して `docker push`、対象PCで `docker pull` します。再現性を優先する運用では、タグだけでなく `docker image inspect --format '{{index .RepoDigests 0}}'` で得たdigestも記録してください。
