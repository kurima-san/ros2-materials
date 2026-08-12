# GLIM単体Docker

CUDA 12.6版GLIMだけを含むMapping用イメージです。MID-360 driverとLocalizationは含みません。ホストにはNVIDIA DriverとNVIDIA Container Toolkitが必要です。

```bash
cd slam-localization-api/lidar_localization_ros2/docker/glim
mkdir -p ~/glim/config ~/ros_bags ~/glim_maps
docker compose build
docker compose run --rm --no-deps glim true
$EDITOR ~/glim/config/glim/config_ros.json  # /livox/lidar, /livox/imuを指定
xhost +local:docker
docker compose up glim
```

センサー単体Dockerと組み合わせる場合、両Composeでhost networkと同じ`ROS_DOMAIN_ID`を使用します。bagを直接処理する場合:

```bash
docker compose run --rm --no-deps glim \
  ros2 run glim_ros glim_rosbag /data/bags/BAG_DIRECTORY \
  --ros-args -p config_path:=/data/config/glim
```

この構成はx86_64向けです。JetsonではJetPack/L4Tに合うイメージを別途用意してください。


## arm64ボード

JetsonおよびQualcomm系ボードは[共通arm64手順](../ARM64_PLATFORMS.md)を確認してください。
