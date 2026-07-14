# 学習ガイド

## この教材の目的

Dockerの個別コマンドを暗記するのではなく、次の流れを理解することを目的としています。

```text
開発環境を定義する
    ↓
Imageを作る
    ↓
Containerを起動する
    ↓
Container内でROS 2を開発する
    ↓
完成版を1操作で起動する
    ↓
Ubuntu・Jetson実機へ展開する
```

## 学習フェーズ

### フェーズ1：Dockerの全体像

- ROS 2開発でDockerを使う理由
- Dockerfile、Image、Container、Composeの関係
- Dockerを操作するTerminalとContainer内Terminalの違い

### フェーズ2：基本操作

- 公開Imageを取得する
- DockerfileからImageを作る
- ImageからContainerを起動する
- Composeで複数のContainerと設定をまとめる

### フェーズ3：ROS 2開発

- GUI付き開発環境を起動する
- PC側のソースをContainerへ共有する
- Container内で`colcon build`とROS 2実行を行う
- 開発用と完成版を分ける

### フェーズ4：通信と展開

- Container間通信
- PCとContainerの通信
- ROS 2のDDS通信
- Windows＋WSL、Ubuntuネイティブ、Jetsonの使い分け

## 最初の実習

```bash
cd examples/01_wsl_ubuntu_desktop
chmod +x *.sh docker/*.sh
./start-dev.sh
```

ブラウザで次を開きます。

```text
http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale
```

## Compose操作はいつ学ぶか

Composeの操作は、Dockerfile、Image、Containerの関係を理解した後に学びます。

- `up`：Composeに書かれたサービスを準備して起動
- `ps`：起動状態を確認
- `exec desktop bash`：起動済みdesktopの内部Terminalを開く
- `logs`：実行中の出力を確認
- `down`：Compose環境を終了して片付ける

これらは教材のSTEP 07で、実行順と対象をまとめて説明しています。
