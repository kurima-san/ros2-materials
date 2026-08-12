# MID-360センサー単体Docker

Livox-SDK2と`livox_ros_driver2`だけを含む、センサー受信用イメージです。GLIMやLocalizationは含みません。

```bash
cd slam-localization-api/lidar_localization_ros2/docker/mid360
mkdir -p ~/mid360/config ~/ros_bags
docker compose build
docker compose run --rm --no-deps mid360 true
$EDITOR ~/mid360/config/MID360_config.json  # PC側IPとLiDAR IPを修正
xhost +local:docker                         # RViz使用時だけ
docker compose up mid360
```

`/livox/lidar`（PointCloud2）と`/livox/imu`をhost network上へPublishします。別コンテナは同じ`ROS_DOMAIN_ID`で購読してください。複数台の場合は`lidar_configs`へ登録し、IP・ポートを重複させないでください。


## arm64ボード

JetsonおよびQualcomm系ボードは[共通arm64手順](../ARM64_PLATFORMS.md)を確認してください。
