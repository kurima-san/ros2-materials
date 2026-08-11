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
3. Odometryと`map`、`odom`、`base_link`のTF構成
4. 2D、3D LiDAR、Visual SLAMの選択
5. LocalizationとNavigation2の接続
6. Bringup、デバッグ、動作確認

## 主な機能

- 章ごとのナビゲーションと完了チェック
- 教材内検索
- 学習テーマによる章の絞り込み
- TF・Topic・データ経路の図解
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
