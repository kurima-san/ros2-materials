# MID-360 + GLIM + lidar_localization_ros2 統合Docker

センサードライバ、CUDA 12.6版GLIM、Localizationを1イメージ・1コンテナへ統合した構成です。個別更新や障害分離を優先する本番運用では各独立Dockerを、導入の簡単さを優先する評価環境では本構成を使います。

## 初回設定と起動

```bash
cd slam-localization-api/lidar_localization_ros2/docker/all-in-one
mkdir -p ~/mid360_stack/config ~/ros_bags ~/ros2_maps
docker compose build
docker run --rm \
  -v "${HOME}/mid360_stack/config:/data/config:rw" \
  mid360-glim-localization:humble-cuda12.6 true

$EDITOR ~/mid360_stack/config/MID360_config.json
$EDITOR ~/mid360_stack/config/glim/config_ros.json
$EDITOR ~/mid360_stack/config/localization.yaml

# localization.yamlのmap_pathは /data/maps/実際の地図.pcd に変更する
xhost +local:docker
docker compose up stack
```

設定ファイルの初期生成にはGPUを使わないため、ここではComposeの`stack` serviceではなく、
build済みimageを`docker run`で直接起動します。このコマンドは`--gpus`を指定しないため、
CDIが未設定でも設定ファイルを生成できます。
`docker compose run ... stack true`を使うと、`true`の実行前にDockerが`stack`のGPUを
割り当てようとして、CDI未設定の環境では `failed to discover GPU vendor from CDI` で
停止します。初期生成には必ず上記の`docker run`を使用してください。

実際の`stack`起動にはNVIDIA GPUが必要です。次の確認が両方成功してから起動します。

```bash
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi
```

2つ目だけ `failed to discover GPU vendor from CDI` で失敗する場合、GPU driverは動作して
いますが、ホスト側のNVIDIA Container Toolkit/CDI設定が完了していません。次をホストで
実行します（コンテナ内では実行しません）。

```bash
# コマンドがなければ、先にNVIDIA Container Toolkitをインストールする
command -v nvidia-ctk

sudo nvidia-ctk runtime configure --runtime=docker
sudo mkdir -p /etc/cdi
sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
sudo systemctl restart docker

nvidia-ctk cdi list
docker run --rm --gpus all nvidia/cuda:12.6.0-base-ubuntu22.04 nvidia-smi
```

最後のDockerコマンドが成功してから `docker compose up stack` を実行してください。
Dockerを再起動すると既存コンテナへ影響するため、実行前に必要なコンテナを停止します。

`lidar_localization_ros2` は更新されるため、Dockerfileでは同梱の巨大な
`CMakeLists.txt` を丸ごと上書きせず、必要な `ndt_omp` リンク修正だけを適用します。
修正スクリプトは行番号や周辺行に依存せず、legacy変数がなければ変更せずに正常終了します。
このため、上流でCMakeの行位置やソース、インストール対象が変更されても、cloneした
リビジョンとの食い違いだけを理由にbuildが停止しません。

ビルドに失敗した場合は、末尾の `exit code: 1` だけでなく、その前にある最初のエラーを
確認できるよう、キャッシュを無効化してプレーンログを保存してください。

```bash
docker compose build --no-cache --progress=plain 2>&1 | tee build.log
```

依存関係の導入と `colcon build` は別レイヤーになっているため、失敗したステップが
`rosdep` とコンパイルのどちらかもログから判別できます。

`ndt_omp_ros2` はHumbleのrosdep databaseに定義がないため、Localizationと同じworkspaceへ
ソースをcloneしてbuildします。また、このイメージは `BUILD_TESTING=OFF` でbuildするので、
Humbleで提供されないテスト専用rosdep keyの `ros_testing` は `rosdep install` の対象から
除外します。環境によってAPT candidateが存在しない `python3-pil` もrosdepから除外し、
同じPython moduleを提供する `Pillow` をpipで導入してimportまでbuild中に確認します。

既定ではMID-360、GLIM、Localizationをすべて起動します。GLIMで地図作成だけを行う場合はLocalizationを無効化します。

```bash
ENABLE_LOCALIZATION=false docker compose up stack
```

既存地図によるLocalizationだけならGLIMを無効化します。

```bash
ENABLE_GLIM=false LOCALIZATION_RVIZ=true docker compose up stack
```

`run-all.sh`は子プロセスのいずれかが終了すると他も停止させるため、一部だけが残って正常に見える状態を避けます。ホストにはNVIDIA DriverとNVIDIA Container Toolkitが必要で、このCUDA repository構成はx86_64向けです。


## arm64ボード

JetsonおよびQualcomm系ボードは[共通arm64手順](../ARM64_PLATFORMS.md)を確認してください。
