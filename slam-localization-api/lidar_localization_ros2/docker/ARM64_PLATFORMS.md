# Jetson・Qualcomm系arm64ボードでDocker環境を作る

Jetson Orinシリーズ、およびUbuntu 22.04を実行できるQualcomm Robotics系ボード（RB5/RB6、QCS6490系など）を対象にします。同じ製品名でもBSP、Ubuntu、GPU runtimeが異なるため、まず実機を確認します。

```bash
uname -m                         # aarch64
cat /etc/os-release             # Ubuntu 22.04
docker version
docker compose version
```

## 対応表

| 構成 | Jetson arm64 | Qualcomm arm64 |
|---|---|---|
| MID-360センサー単体 | 対応。実機でnative build | 対応。実機でnative build |
| Localization単体 | 対応候補。実bagで性能確認 | 対応候補。実bagで性能確認 |
| CUDA 12.6 GLIM単体 | 同梱Dockerfileは非対応 | 非対応（CUDA GPUではない） |
| CUDA 12.6オールインワン | 同梱Dockerfileは非対応 | 非対応 |

「対応候補」はarm64向けROS依存パッケージが使用中のOS repositoryに揃う場合にbuildできるという意味です。buildできても点群レート、温度、メモリ、Localization周期を実データで確認してください。

## 1. Jetsonの準備

1. 対象機種をサポートするJetPackをNVIDIA公式手順で導入します。
2. JetPackに対応するNVIDIA Container Runtimeを有効にします。
3. Ubuntu 22.04 / ROS 2 Humbleを使うため、対応するJetPack 6系を選びます。

```bash
cat /etc/nv_tegra_release
sudo docker info | sed -n '/Runtimes/,+2p'
sudo docker run --rm --runtime nvidia ubuntu:22.04 \
  sh -c 'test -e /dev/nvhost-ctrl-gpu && echo NVIDIA-runtime-visible'
```

最後はデバイス公開の確認であり、GLIM動作保証ではありません。JetsonではホストのJetPack/L4TとコンテナのCUDA/cuDNN/TensorRTを対応させます。x86_64用CUDA repositoryを追加してはいけません。

## 2. Qualcomm系ボードの準備

ベンダーBSPでUbuntu 22.04 arm64が提供され、Docker Engineを実行できる構成を使用します。Qualcomm GPU/NPUはNVIDIA CUDAではありません。同梱GLIMイメージの`--gpus all`、CUDA 12.6、Koide CUDA PPAをAdreno/Hexagon向けに置き換えることはできません。

```bash
uname -m
cat /etc/os-release
ip link
docker run --rm arm64v8/ubuntu:22.04 uname -m
```

ベンダー提供のGPU/NPU container runtimeもCUDA互換を意味しません。このガイドではQualcomm accelerator上のGLIM buildをサポート対象にしません。

## 3. MID-360センサー単体をnative buildする（共通）

`ros:humble-ros-base-jammy`はmulti-architectureイメージなので、arm64ボード上のbuildではarm64 layerが選択されます。最初はクロスbuildではなく実機native buildを推奨します。

```bash
cd slam-localization-api/lidar_localization_ros2/docker/mid360
mkdir -p ~/mid360/config ~/ros_bags
docker compose build --pull
docker image inspect livox-mid360:humble \
  --format '{{.Architecture}}/{{.Os}}'       # arm64/linux

docker compose run --rm --no-deps mid360 true
$EDITOR ~/mid360/config/MID360_config.json
docker compose up mid360
```

MID-360とボードの有線NICを同一subnetに設定します。host networkを使うため、NICはコンテナ内ではなくホストのNetworkManagerまたはnetplanで設定します。

```bash
docker compose exec mid360 ros2 topic type /livox/lidar
docker compose exec mid360 ros2 topic hz /livox/lidar
docker compose exec mid360 ros2 topic hz /livox/imu
```

## 4. Localization単体をnative buildする（共通）

```bash
cd slam-localization-api/lidar_localization_ros2/docker
mkdir -p ~/ros2_maps ~/ros_bags
docker compose build --pull localization
docker image inspect lidar-localization-mid360:humble \
  --format '{{.Architecture}}/{{.Os}}'       # arm64/linux
docker compose run --rm localization
```

画面のないボードではComposeの`/tmp/.X11-unix` volumeを削除するかRVizを起動しないlaunch設定にします。NDTのthread数、voxel size、入力点数を調整し、CPU温度とLocalization周期を測定します。

## 5. JetsonでGLIMを使う場合

同梱`docker/glim/Dockerfile`と`docker/all-in-one/Dockerfile`はx86_64用CUDA 12.6 APT repositoryとGLIM PPAを使うため、arm64 buildを明示的に停止します。Jetson版には次を満たす別Dockerfileが必要です。

1. JetPack/L4T releaseと一致するNVIDIA L4T CUDA base imageを使う。
2. GTSAM、gtsam_points、GLIMをarm64上でsource buildする。
3. CUDA architectureを搭載GPUに合わせる。
4. ROS 2 Humble workspaceとABIを揃える。
5. GLIMと実bagでCUDA利用を検証する。

JetPack/L4Tの組み合わせは機種ごとに変わるため、未検証の固定base tagは提示しません。NVIDIAのJetPack release notesとNGCのL4T container tagを照合して固定してください。buildx/QEMUではCUDA実行テストができないため、最初はJetson実機でbuildします。

## 6. QualcommでMappingする推奨構成

QualcommボードではMID-360受信とLocalizationを実行し、GLIM MappingはCUDA対応x86_64 PCへ分離する構成を推奨します。

```text
MID-360
  └─ Qualcomm arm64: docker/mid360（収録・Publish）
       ├─ rosbagをSSDへ保存 → x86_64 GPU PCのdocker/glimでMapping
       └─ 作成済みPCD → Qualcomm arm64のdocker/でLocalization
```

PC間のリアルタイムROS 2通信では同一`ROS_DOMAIN_ID`に加え、multicast、firewall、DDS interface設定を確認します。現場ではbagを移送してMappingする方が再現しやすい構成です。

## 7. arm64イメージの配布

arm64ボード上でbuildしたイメージをarm64配布先へ渡します。

```bash
docker save livox-mid360:humble | gzip > livox-mid360_humble_arm64.tar.gz
sha256sum livox-mid360_humble_arm64.tar.gz > livox-mid360_humble_arm64.tar.gz.sha256

docker save lidar-localization-mid360:humble | gzip \
  > lidar-localization-mid360_humble_arm64.tar.gz
sha256sum lidar-localization-mid360_humble_arm64.tar.gz \
  > lidar-localization-mid360_humble_arm64.tar.gz.sha256
```

配布記録にはボード名、BSP/JetPack、Ubuntu、kernel、イメージID、Git commit、SHA-256を残します。Jetson用とQualcomm用は同じarm64でも、vendor libraryやdevice mountを含む場合は相互互換とみなしません。
