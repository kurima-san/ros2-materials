# ROS 2 × Docker 基礎から実機開発まで

Dockerを初めて使う人が、Dockerの全体像からROS 2の開発・通信・Ubuntu／Jetson展開まで順番に学べるローカルWeb教材です。

## 教材を開く

Windows:

```text
serve.bat
```

WSL／Ubuntu:

```bash
chmod +x serve.sh
./serve.sh
```

または`index.html`をブラウザで直接開きます。

## 学習の順番

1. Dockerを使う理由
2. Dockerfile、Image、Container、Composeの関係
3. Dockerを操作する場所
4. `pull`、`build`、`run`による基本操作
5. Dockerfileによる環境作成
6. Composeによる開発環境の構成
7. Composeの起動・確認・内部操作・終了
8. 開発用と完成版の分離
9. GUI付きROS 2サンプル
10. Container内でのROS 2開発
11. 完成版Imageと自動起動
12. Docker通信とROS 2通信
13. WSL、Ubuntu、Jetsonへの展開

## 標準サンプル

```bash
cd examples/01_wsl_ubuntu_desktop
chmod +x *.sh docker/*.sh
./start-dev.sh
```

起動後、ブラウザで次を開きます。

```text
http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
```

初期VNCパスワードは`ros2`です。

## 同梱サンプル

- `examples/01_wsl_ubuntu_desktop`：GUI、RViz2、疑似LiDAR、複数Container
- `examples/02_wsl_jetson_crossdev`：Jetson実機なしのARM64確認
- `examples/03_ubuntu_native_network`：UbuntuネイティブのROS 2外部通信

## 最初に使う補助スクリプト

```bash
./start-dev.sh       # 開発環境を起動
./status-dev.sh      # 状態を表示
./open-terminal.sh   # desktop Container内のTerminalを開く
./logs-dev.sh        # ログを表示
./stop-dev.sh        # 開発環境を終了
```

Windows用の`.bat`ファイルも同じフォルダーに入っています。
