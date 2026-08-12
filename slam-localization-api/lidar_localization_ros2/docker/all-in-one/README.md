# MID-360 / MID-360S + GLIM + lidar_localization_ros2 オールインワンイメージ

1つのイメージにMID-360/MID-360S driver、GLIM、Localizationの3機能を収録しています。ただし、実行単位は次のどちらかです。

- **Mapping:** MID-360またはMID-360S（またはbag再生）+ GLIM
- **Localization:** MID-360またはMID-360S（またはbag再生）+ lidar_localization_ros2

GLIMとLocalizationを同時には起動しません。Compose service名も用途を表す`mapping`と`localization`です。

## Buildと初期設定

```bash
cd slam-localization-api/lidar_localization_ros2/docker/all-in-one
mkdir -p ~/mid360_stack/config ~/ros_bags ~/ros2_maps
docker compose build

# GPU不要の設定ファイル初期生成
docker run --rm \
  -v "${HOME}/mid360_stack/config:/data/config:rw" \
  mid360-glim-localization:humble-cuda12.6 true

$EDITOR ~/mid360_stack/config/MID360s_config.json  # MID-360S実機
# MID-360を使う場合: $EDITOR ~/mid360_stack/config/MID360_config.json
$EDITOR ~/mid360_stack/config/glim/config_ros.json
$EDITOR ~/mid360_stack/config/localization.yaml
# localization.yamlのmap_pathを /data/maps/地図名.pcd にする
```

完全にbuildし直す場合:

```bash
docker compose down --remove-orphans
docker compose build --no-cache --pull --progress=plain 2>&1 | tee build.log
```

## センサーモデルを選ぶ

既定値は実機に合わせて`mid360s`です。MID-360SとMID-360の両方の設定ファイルを生成し、`SENSOR_MODEL`で選択します。Livox driverのlaunch名は両機種ともupstreamの`rviz_MID360_launch.py`です。各JSONのLiDAR IPを実機に合わせてください。

```bash
# MID-360S（既定）
docker compose up mapping

# MID-360
SENSOR_MODEL=mid360 docker compose up mapping
```

同じ選択方法を`localization`にも使用できます。bag再生時はセンサードライバを起動しないため、`SENSOR_MODEL`は使用しません。

## リアルタイム実行

```bash
xhost +local:docker

# MID-360S + GLIM（地図作成、既定）
docker compose up mapping

# または、MID-360S + Localization（既存地図で自己位置推定）
docker compose up localization

# MID-360 + Localization
SENSOR_MODEL=mid360 docker compose up localization
```

同時には起動しません。停止は`Ctrl+C`です。地図や設定はそれぞれ`~/ros2_maps`、`~/mid360_stack/config`に残ります。

### リアルタイム入力をbagにも保存する

```bash
# Mapping入力を記録しながら処理
RECORD_BAG=true docker compose up mapping

# またはLocalization入力を記録しながら処理
RECORD_BAG=true docker compose up localization
```

bagはホストの`~/ros_bags`へ保存されます。記録対象は`/livox/lidar`、`/livox/imu`、`/tf`、`/tf_static`です。ホストに同じROS 2環境があればホスト側の`ros2 bag record`でも記録できますが、必須ではありません。

記録topicを指定する場合は、空白を入れないカンマ区切りで`RECORD_TOPICS`を渡します。

```bash
RECORD_BAG=true \
RECORD_TOPICS=/livox/lidar,/livox/imu,/tf,/tf_static,/diagnostics \
  docker compose up localization
```

## rosbagから再現する

まずホストから見えるbag pathを確認します。

```bash
find ~/ros_bags -maxdepth 2 -name metadata.yaml -print
```

`BAG_PATH`はコンテナ内のpath（`/data/bags/...`）で指定します。

```bash
# bag + GLIM（Mapping再現）
INPUT_MODE=bag BAG_PATH=/data/bags/BAG_DIRECTORY \
  docker compose up mapping

# bag + Localization（Localization再現）
INPUT_MODE=bag BAG_PATH=/data/bags/BAG_DIRECTORY \
  LOCALIZATION_RVIZ=true docker compose up localization

# bag内の指定topicだけを再生（空白なしのカンマ区切り）
INPUT_MODE=bag BAG_PATH=/data/bags/BAG_DIRECTORY \
PLAY_TOPICS=/livox/lidar,/livox/imu,/tf,/tf_static \
  docker compose up localization
```

`PLAY_TOPICS`を省略するとbag内の全topicを再生します。bag modeではMID-360/MID-360S driverを起動せず、コンテナ内で`ros2 bag play --clock`を実行し、処理nodeへ`use_sim_time=true`を渡します。したがってホスト側で別途bagをplayする必要はありません。MappingまたはLocalizationに必要な`/livox/lidar`と`/livox/imu`を除外しないでください。TFも再現する場合は`/tf`と`/tf_static`を含めます。

## GPU/CDI確認

```bash
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi
```

2つ目だけ`failed to discover GPU vendor from CDI`になる場合:

```bash
command -v nvidia-ctk
sudo nvidia-ctk runtime configure --runtime=docker
sudo mkdir -p /etc/cdi
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
sudo systemctl restart docker
nvidia-ctk cdi list
```

Docker再起動後、GPUテストを再実行してください。ホストにはNVIDIA DriverとNVIDIA Container Toolkitが必要です。このイメージはx86_64/CUDA 12.6向けです。

## Build実装上の注意

`ndt_omp_ros2`はHumbleのrosdep databaseに定義がないため同じworkspaceへcloneしてbuildします。上流`CMakeLists.txt`は丸ごと置換せず、対象targetのlegacy link変数だけを修正します。`BUILD_TESTING=OFF`のため`ros_testing`を、APT candidateが不安定な`python3-pil`をrosdepから除外し、Pillowをpipで導入してimport確認します。

## arm64ボード

JetsonおよびQualcomm系ボードは[共通arm64手順](../ARM64_PLATFORMS.md)を確認してください。
