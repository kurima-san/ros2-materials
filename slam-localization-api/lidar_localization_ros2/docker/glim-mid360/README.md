# GLIM + Livox MID-360 Docker 環境

Ubuntu 22.04 / ROS 2 Humble、CUDA 12.6版GLIM、Livox-SDK2、
`livox_ros_driver2` を1つのイメージに構築します。MID-360を複数台使う場合も、
同じ `MID360_config.json` の `lidar_configs` に各センサーを登録します。

## ホストの準備

NVIDIAドライバ、Docker Engine、Compose plugin、NVIDIA Container Toolkitが必要です。
ホストのCUDA Toolkitは不要ですが、`nvidia-smi` と次のGPUテストが成功することを確認します。

```bash
nvidia-smi
docker run --rm --gpus all ubuntu:22.04 nvidia-smi
mkdir -p ~/glim_mid360/config ~/ros_bags ~/glim_maps
```

## buildと初回設定

```bash
cd slam-localization-api/lidar_localization_ros2/docker/glim-mid360
docker compose build

# 初回だけ公式サンプル設定をホスト側へ生成する
docker compose run --rm --no-deps mid360 true
$EDITOR ~/glim_mid360/config/MID360_config.json
$EDITOR ~/glim_mid360/config/glim/config_ros.json
```

`MID360_config.json` のPC側IPと各LiDAR IPを実ネットワークに合わせます。
複数台なら `lidar_configs` 配列へ機器を追加し、IPや使用ポートが衝突しないようにします。
ComposeはPointCloud2を出力する `rviz_MID360_launch.py` を起動します。
GLIMの `config_ros.json` は通常 `points_topic=/livox/lidar`、
`imu_topic=/livox/imu` にします。複数台の点群を1つのtopicへ単純に混ぜず、
時刻同期・外部標定を行った統合topicをGLIMへ指定してください。

## リアルタイム起動

```bash
xhost +local:docker                         # GUIを使うLinuxホストだけ
docker compose up mid360 glim
```

別端末から確認する場合:

```bash
docker compose exec mid360 ros2 topic list
docker compose exec mid360 ros2 topic type /livox/lidar
docker compose exec glim nvidia-smi
```

終了は `Ctrl+C`、GUI許可を戻す場合は `xhost -local:docker` です。MID-360のUDPと
ROS 2 DDSのためhost networkを使用します。設定、bag、地図はそれぞれホストの
`~/glim_mid360/config`、`~/ros_bags`、`~/glim_maps` に残ります。

## rosbagからGLIMを実行

リアルタイム用serviceを止め、shellから `glim_rosbag` を実行します。

```bash
docker compose run --rm --no-deps glim bash
ros2 run glim_ros glim_rosbag /data/bags/BAG_DIRECTORY \
  --ros-args -p config_path:=/data/config/glim
```

上の `BAG_DIRECTORY` は実際のbagディレクトリ名へ置き換え、余分な空白を入れず
`/data/bags/BAG_DIRECTORY` と指定してください。

## 別PCへオフライン配布

CUDA 12.6パッケージはこの構成ではx86_64向けです。配布先にも互換性のあるNVIDIA
ドライバとNVIDIA Container Toolkitが必要です。

```bash
docker image inspect glim-mid360:humble-cuda12.6 >/dev/null
docker save glim-mid360:humble-cuda12.6 | gzip > glim-mid360_humble-cuda12.6.tar.gz
sha256sum glim-mid360_humble-cuda12.6.tar.gz > glim-mid360_humble-cuda12.6.tar.gz.sha256

# 対象PC
sha256sum -c glim-mid360_humble-cuda12.6.tar.gz.sha256
gunzip -c glim-mid360_humble-cuda12.6.tar.gz | docker load
docker compose -f compose.yaml up mid360 glim
```

`tar.gz`、SHA-256ファイル、`compose.yaml` を一緒にコピーしてください。設定ファイルは
機器・PCごとに調整し、イメージへ秘密情報や収録データを焼き込まない運用にします。
