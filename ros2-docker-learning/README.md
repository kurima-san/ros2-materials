# ROS 2 × Docker 開発・配布実践コース（更新版）

## 開く

- `index.html` を直接開く
- Windows: `serve.bat`
- WSL / Ubuntu: `chmod +x serve.sh && ./serve.sh`

## 同梱サンプル

1. `examples/01_wsl_ubuntu_desktop`  
   noVNCデスクトップ、RViz2、疑似LiDAR、dev/runtime分離
2. `examples/02_wsl_jetson_crossdev`  
   Jetson実機なしでlinux/arm64 build・実行
3. `examples/03_ubuntu_native_network`  
   Ubuntuネイティブのhost network＋ROS 2 DDS

## 推奨開始点

```bash
cd examples/01_wsl_ubuntu_desktop
chmod +x *.sh docker/*.sh
./start-dev.sh
```

初期VNCパスワードは `ros2` です。必要に応じて `.env` で変更してください。

## 注意

Docker Engineをこの生成環境で実行できなかったため、Docker imageの実buildは未実施です。HTML、YAML、Python、shell、JSONの静的検証を行っています。
