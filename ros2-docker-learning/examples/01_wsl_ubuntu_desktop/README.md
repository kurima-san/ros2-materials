# WSL＋Ubuntu／Ubuntu：ブラウザDesktop開発サンプル

このサンプルでは、Dockerコマンドを直接覚える前に補助スクリプトから始められます。

## 1. 最初に確認する場所

次の操作は **Containerの外側**、つまりWSL Ubuntu Terminal、Ubuntu Terminal、PowerShellまたはコマンドプロンプトで行います。

```bash
docker version
docker compose version
docker run --rm hello-world
```

## 2. 開発環境を起動する

### WSL／Ubuntu

```bash
chmod +x *.sh docker/*.sh
./start-dev.sh
```

### Windows

`start-dev.bat`をダブルクリックします。

- URL: http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
- VNC password: `ros2`
- RViz2、疑似LiDAR、監視ノードが起動します。

## 3. Container Terminalを開く

### WSL／Ubuntu

```bash
./open-terminal.sh
```

### Windows

`open-terminal.bat`をダブルクリックします。

これは、起動済みの`desktop` Container内で`bash`を開きます。全Containerを起動する操作ではありません。

Container内では次を試せます。

```bash
cd /workspace
ros2 topic list
ros2 topic echo /scan --once
colcon build --symlink-install
source install/setup.bash
```

Container内から出るには次を実行します。

```bash
exit
```

## 4. 状態とログを見る

```bash
./status-dev.sh
./logs-dev.sh
```

Windowsでは`status-dev.bat`、`logs-dev.bat`を使用します。

## 5. 終了する

```bash
./stop-dev.sh
```

Windowsでは`stop-dev.bat`を使用します。

## スクリプトとDockerコマンドの対応

| スクリプト | 内部で行う操作 | 意味 |
|---|---|---|
| `start-dev` | `docker compose ... up -d --build` | Imageを作り、YAMLのサービスを起動 |
| `open-terminal` | `docker compose ... exec desktop bash` | 起動済みdesktopの中でbashを開く |
| `status-dev` | `docker compose ... ps` | 状態を表示 |
| `logs-dev` | `docker compose ... logs -f` | ログを表示 |
| `stop-dev` | `docker compose ... down` | 停止してContainerとNetworkを削除 |

## 開発版と完成版

`compose.dev.yaml`はブラウザDesktop、ソース共有、疑似センサーを含む開発用です。

`compose.runtime.yaml`はGUIやソース共有を持たず、完成済みROS 2システムを自動起動する実行用です。

```bash
./start-runtime.sh
```

> `compose.network.yaml`は通信だけを分離して試す発展用サンプルです。最初は使用しなくて構いません。
