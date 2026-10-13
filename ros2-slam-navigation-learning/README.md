# ROS 2 SLAM・Localization・Navigation2 学習教材

移動ロボットにおけるSLAM（地図作成）、Localization（自己位置推定）、Navigation（自律移動）の役割を区別し、センサー、Odometry、TF、Navigation2、ロボット制御をROS 2上で接続する仕組みを学ぶオフラインHTML教材です。

## 教材を開く

最も簡単な方法は、`OPEN_THIS_STANDALONE.html`をブラウザで開くことです。

- インターネット接続は不要です。
- CSS、JavaScript、Mermaid図を1ファイルに含みます。
- 章の完了状態と表示テーマは、ブラウザ内に保存されます。
- 「印刷・PDF保存」から教材全体をPDF化できます。

フォルダー構成のまま使う場合は、`index.html`を開いてください。

## 学習内容

1. SLAM、Localization、Navigationの目的と入出力
2. 地図作成段階と通常運用段階のアプリケーション構成
3. Odometryの必要性、TFの配信担当、計測からNav2制御までの時系列
4. 2D、3D LiDAR、Visual SLAMの選択
5. LocalizationとNavigation2の接続
6. 実機歩行ロボットでの3D LiDAR障害物回避、安全監視、歩容制御接続
7. Bringup、デバッグ、段階試験

### オドメトリの意味から理解する

[第2章：オドメトリ・TFの理解](index.html#odometry-guide)は、次の疑問を順に解消します。単一ファイル版にも同じ内容を収録しています。

1. `map → odom`と`odom → base_link`は、誰がどの周期で出すのか
2. オドメトリはいつからの移動を表し、いつリセットするのか
3. LiDARの計測時刻と結果のPublish時刻をどう区別するのか
4. 自己位置推定がジャンプしたとき、何が変わり、何を連続して扱えるのか
5. Nav2の局所コストマップ、経路追従、現在速度にどう活用するのか
6. 車輪が回らず機体だけが滑った場合、何が直り、何が直らないのか

50 Hzのオドメトリ、10 HzのLiDAR、60 msの計算時間を仮定した時系列表と、壁・遠方の経路点・滑りの数値例で説明します。数値は説明用で、実機の測定値ではありません。

## 主な機能

- 章ごとのナビゲーションと完了チェック
- 教材内検索
- 学習テーマによる章の絞り込み
- TF・Topic・データ経路の図解
- オドメトリの非同期処理、地図補正、局所観測の保持を追う数値例
- 3D点群のSTVL／Voxel Costmap設定、停止距離、実機試験手順
- 2D、3D LiDAR、Visual SLAMの比較表
- コマンドのコピーボタン
- ダーク・ライト表示
- 理解度確認問題
- 印刷・PDF保存向けレイアウト

## 収録ファイル

| ファイル | 内容 |
|---|---|
| `OPEN_THIS_STANDALONE.html` | 単体で開ける完成版教材 |
| `index.html` | `assets`フォルダを参照する通常版 |
| `COURSE_SOURCE.md` | 教材本文のMarkdown |
| `assets/styles.css` | レイアウトと印刷用スタイル |
| `assets/app.js` | 検索、進捗、コピー、クイズなど |
| `assets/mermaid.min.js` | 接続図のオフライン描画 |
| `course-template.html` | 教材HTMLを再生成するためのPandocテンプレート |
| `licenses` | 同梱フォント・描画ライブラリのライセンス |
| `VERSION.txt` | ビルド情報 |

## 対象

- Ubuntu 22.04
- ROS 2 Humble
- 2D LiDAR SLAM / Localization / Navigation2
- 3D LiDAR / LIO / 3D Graph SLAM
- Visual SLAM / VO / VIO
- Livox MID-360の手持ちMapping
- 四足歩行ロボット、ヒューマノイド、車輪移動ロボット
- 3D LiDARを使う実機の障害物回避と安全停止
