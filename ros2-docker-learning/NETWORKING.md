# Docker / ROS 2 通信チートシート

|通信|設定|
|---|---|
|container間|同一Compose network。通常アプリはservice名で接続|
|host→container|`ports: ["127.0.0.1:8080:80"]`|
|container→host|Docker Desktopは`host.docker.internal`。Linux Engineは`extra_hosts: host-gateway`|
|外部PC→container|host IP＋published port＋firewall|
|ROS 2 DDSを外部LANへ|Ubuntuネイティブで`network_mode: host`を第一候補|
|Fast DDS SHMをhost networkで共有|`ipc: host`も指定|
|multicast不可|Discovery Server、explicit peers、DDS Router/Zenoh等を検討|

WSL＋Docker Desktopは開発と単体試験に便利ですが、外部ロボットとのDDS最終試験はUbuntuネイティブまたはJetsonで行います。
