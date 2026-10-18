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
5. LocalizationとNavigation2の接続、大域・局所コストマップ、障害物回避の仕組み
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

### コストマップから障害物回避まで理解する

[第6章：コストマップ](index.html#costmap-guide)では、次の流れを図・比較表・通路の具体例で説明します。単一ファイル版にも同じ内容を収録しています。

6.4節の冒頭には、同じ部屋を「実空間 → 障害物セル → 周囲へ広げたコスト」の3枚で比較する図と、台車へ近づく6セルの数値を拡大した図を収録しています。

6.5節では、大域の全体図と局所窓の拡大図で、同じコストを経路選択と直近の速度判断へ使う違いを説明します。6.10節では、局所で閉塞を検知して停止する状態、大域に未反映の状態、大域にも反映して別通路へ再計画する状態を並べています。

地図関連の図は次の場所から読めます。

| 節 | 図で理解する内容 |
|---|---|
| 6.4 | 3D地図・2D地図の利用先、実空間とコストの格子 |
| 6.5 | 大域と局所、移動窓、PlannerとControllerの関係 |
| 6.6 | 保存地図・観測レイヤー・合成結果・Inflationの4段階 |
| 6.7 | 光線による登録と消去、新しい観測がない場合 |
| [6.8](index.html#costmap-radius-example) | 半径0.45 m・膨張0.80 m・係数3.0の数値例と通路幅、膨張範囲の重なり、距離コスト曲線 |
| 6.10 | 局所での停止と、大域への反映・再計画 |
| [6.13～6.15](index.html#costmap-parameters) | 主要パラメータ、広場・狭路の速度変化、RPP・Speed Filter・動的Inflation |
| 11.3 | STVLの点群・3Dボクセル・2D投影、時間減衰、再登録、視野モデル |

1. 占有地図とコストマップの違い、内部コスト値の意味
2. 大域での通路選択と局所での速度判断、mapとodomの使い分け
3. Static・Obstacle・Voxel・STVL・Inflationの役割
4. 障害物の登録・消去と、未観測領域の扱い
5. Footprint、Inflation、解像度と衝突判定
6. Planner・Controller・再計画の処理ループと独立した周期
7. 台車を局所的に避ける場合と、通路閉塞で大域的に迂回する場合
8. `inflation_radius`と`cost_scaling_factor`の違い、狭路の通行可否と速度調整
9. RPPの近接減速、区域ごとの速度制限、走行中のInflation変更の範囲と条件

## 主な機能

- 章ごとのナビゲーションと完了チェック
- 教材内検索
- 学習テーマによる章の絞り込み
- TF・Topic・データ経路の図解
- オドメトリの非同期処理、地図補正、局所観測の保持を追う数値例
- 大域・局所コストマップ、障害物レイヤー、経路計画と速度制御の図解
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
| `assets/figures/*.svg` | コストマップ・レイヤー合成・障害物の消去・STVLの図 |
| `scripts/generate_costmap_figures.py` | コストマップ図の再生成（Python標準ライブラリのみ） |
| `scripts/generate_costmap_scope_figures.py` | 大域・局所と再計画の図の再生成（Python標準ライブラリのみ） |
| `scripts/generate_map_lifecycle_figures.py` | レイヤー合成・観測消去・移動窓・STVLの図の再生成 |
| `scripts/generate_costmap_tuning_figures.py` | Inflationの比較・距離コスト曲線・狭路の速度変化の図の再生成 |
| `scripts/generate_costmap_radius_example.py` | 半径0.45 mの数値例と通路幅比較の図の再生成（NumPy・Matplotlib・日本語フォントが必要） |
| `course-template.html` | 教材HTMLを再生成するためのPandocテンプレート |
| `licenses` | 同梱フォント・描画ライブラリのライセンス |
| `VERSION.txt` | ビルド情報 |

コストマップ図を変更するときは、教材フォルダーで`python3 scripts/generate_costmap_figures.py`を実行します。大域・局所と再計画の図は`python3 scripts/generate_costmap_scope_figures.py`で再生成します。図は通常版ではSVGファイルを参照し、単一ファイル版では同じSVGをデータURLとして内包しています。再生成後は単一ファイル版の対応する画像データも更新します。

レイヤー・消去・STVLの図は`python3 scripts/generate_map_lifecycle_figures.py`で再生成します。SVGは拡大と印刷に対応し、スマートフォンでは比較図を縦に並べます。

Inflationと速度の図は`python3 scripts/generate_costmap_tuning_figures.py`で再生成します。Humbleの式に沿った説明図で、実機やNav2シミュレーションの計測結果ではありません。

半径0.45 mの2枚の図は`python3 scripts/generate_costmap_radius_example.py`で再生成します。NumPy、Matplotlib、Ubuntu/Debianの`fonts-noto-cjk`を使用し、日本語の文字形をSVG内へ埋め込むため、閲覧側でのフォント追加は不要です。再生成したSVGは単一ファイル版にも反映します。

## 対象

- Ubuntu 22.04
- ROS 2 Humble
- 2D LiDAR SLAM / Localization / Navigation2
- 3D LiDAR / LIO / 3D Graph SLAM
- Visual SLAM / VO / VIO
- Livox MID-360の手持ちMapping
- 四足歩行ロボット、ヒューマノイド、車輪移動ロボット
- 3D LiDARを使う実機の障害物回避と安全停止
