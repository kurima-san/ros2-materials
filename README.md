# ROS 2 学習資料

ROS 2の環境構築、センシング、地図作成、自己位置推定、自律移動に関する学習資料をまとめたリポジトリです。

## 教材一覧

| フォルダー | 内容 | 開始ファイル |
|---|---|---|
| [`ros2-docker-learning`](ros2-docker-learning/) | Dockerを利用したROS 2開発環境 | `README.md` |
| [`ros2-slam-navigation-learning`](ros2-slam-navigation-learning/) | SLAM、Localization、Navigation2の役割と接続 | `README.md` |
| [`slam-localization-api`](slam-localization-api/) | SLAM・Localization APIの比較と実装例 | 各サブフォルダーの資料 |
| [`ptz-cam`](ptz-cam/) | PTZカメラとROS 2の接続 | 各Markdown資料 |
| [`python-virtual-env`](python-virtual-env/) | ROS 2でのPython仮想環境 | 各Markdown資料 |
| [`wsl`](wsl/) | WSL上でROS 2を使う際の設定とトラブルシュート | 各Markdown資料 |

## フォルダー命名ルール

- 教材フォルダーは、内容が分かる英小文字の**ケバブケース**（例：`ros2-docker-learning`）に統一します。
- ROS 2を主題とするコースは`ros2-`、学習コースは`-learning`を名前に含めます。
- ROSパッケージなど、外部ツールの規約が優先されるフォルダーは、その規約に従います。
- 空白、日本語、大文字、単語区切りのアンダースコアは教材フォルダー名に使用しません。

## 教材のフォーマットとトーン

- 各教材の入口を`README.md`とし、「概要」「開き方」「学習内容」「対象環境」の順で案内します。
- ローカルWeb教材は`index.html`を通常版、`OPEN_THIS_STANDALONE.html`を単一ファイル版とします。
- 日本語の句読点は「、」「。」を使用し、専門用語は初出時に英語表記と目的を併記します。
- 最初にアプリケーション全体と各機能の責務を示し、その後に仕組み、実習、発展内容を説明します。
- 見出し、カード、表、図では同じ用語を使い、入力・処理・出力の関係が追える見た目にします。

