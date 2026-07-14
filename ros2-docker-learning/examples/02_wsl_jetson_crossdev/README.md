# WSL＋Jetson想定：実機なしARM64 cross development

このサンプルはDocker DesktopのBuildx/QEMUを使い、Jetsonと同じ `linux/arm64` 向けにCPU側ROS 2 packageをbuild・実行します。

## 実行

WSL:

```bash
chmod +x *.sh docker/*.sh
./run-jetson-sim.sh
```

Windows:

```text
run-jetson-sim.bat
```

ログに `machine=aarch64` または `machine=arm64` と、疑似LaserScanの最短距離が表示されます。

## 実機なしで確認できること

- ARM64向けDocker build
- apt/ROS packageがARM64に存在するか
- Python/C++コードのarchitecture依存
- ROS 2 topic、launch、疑似sensor処理

## ローカルARM64 imageだけをbuild

```bash
./build-multiarch.sh
```

複数architectureを1タグでRegistryへpushする例は `push-multiarch-example.sh` です。

## 実機が必要なこと

- CUDA / TensorRT / cuDNN
- Jetson camera、NVENC/NVDEC、VPI、GPIO
- JetPack / Jetson Linux driverとの整合性
- 性能、消費電力、thermal throttling

`docker/Dockerfile.jetson-gpu.template` は実機用の雛形です。`JETSON_BASE_IMAGE` は対象JetPackに合うNVIDIA公式imageを明示してください。固定の最新タグを無条件に使わないでください。

> QEMU実行は非常に遅い場合があります。これは互換性確認用で、性能評価用ではありません。
