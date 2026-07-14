# Ubuntuネイティブ：ROS 2 host networkサンプル

Linux Docker Engine上で、外部ROS 2参加者と通信しやすい `network_mode: host` を試すサンプルです。

```bash
chmod +x *.sh docker/*.sh
./start-host-network.sh
```

別PCまたはhost側で同じ `ROS_DOMAIN_ID=42` を設定し、`ros2 topic list` と `/scan` を確認します。

Fast DDSのshared memoryをcontainer間で使うため、`ipc: host` も指定しています。

> Linuxネイティブ向けです。Docker Desktopのhost networkはLinux Engineと同一ではありません。
