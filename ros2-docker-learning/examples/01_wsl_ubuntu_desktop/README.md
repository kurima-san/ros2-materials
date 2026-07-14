# WSL＋Ubuntu／Ubuntuネイティブ：ブラウザDesktop開発サンプル

## 起動

WSLまたはUbuntu:

```bash
chmod +x *.sh docker/*.sh
./start-dev.sh
```

Windowsから直接起動する場合は `start-dev.bat` を実行します。

- URL: http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
- VNC password: `ros2`
- RViz2は自動起動します。
- `fake_lidar` と `monitor` は別containerで自動起動します。
- `compose.yaml` が標準の開発構成です。

## Desktop内で開発

XFCE Terminalを開きます。

```bash
cd /workspace
colcon build --symlink-install
source install/setup.bash
ros2 topic list
ros2 topic echo /scan --once
```

## 完成版

```bash
./start-runtime.sh
# または
docker compose -f compose.runtime.yaml up --build
```

## 通信だけを分離して確認

```bash
docker compose -f compose.network.yaml up --build
```

## 停止

```bash
./stop-dev.sh
```

> noVNCは学習・開発用です。本番用runtime imageにはGUIを含めていません。
