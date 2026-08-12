# MOLA + MID-360 Docker 環境

ガイドのsource buildと `mola_mid360_tools` を、Ubuntu 22.04 / ROS 2 Humbleイメージ内に構築します。

```bash
cd slam-localization-api/mola/docker
docker compose build
xhost +local:docker                 # GUIを使うLinuxホストだけ
docker compose run --rm mola
ros2 run mola_mid360_tools check_mid360_topics
```

ホストの `~/mola_maps` と `~/ros_bags` はコンテナ内の `/data/maps` と `/data/bags` です。Livox UDPとホスト上のROS 2ノードを扱うためhost networkを使います。

## 別PCへオフライン配布

作成側と対象側のCPUアーキテクチャを合わせてください（amd64同士、arm64同士）。作成側:

```bash
docker image inspect mola-mid360:humble >/dev/null
docker save mola-mid360:humble | gzip > mola-mid360_humble.tar.gz
sha256sum mola-mid360_humble.tar.gz > mola-mid360_humble.tar.gz.sha256
```

`tar.gz`、`sha256`、この `compose.yaml` を対象PCへコピーし、対象PCで:

```bash
sha256sum -c mola-mid360_humble.tar.gz.sha256
gunzip -c mola-mid360_humble.tar.gz | docker load
docker compose -f compose.yaml run --rm mola
ros2 pkg prefix mola_lidar_odometry
```

レジストリ配布では `docker tag` / `docker push` / `docker pull` を使用します。本番記録には可変タグだけでなく、取得したimage digestも残してください。
