# Go2 MID 360のCostmap高さ不整合 調査と案Aの検証

調査日：2026-11-23。対象：ROS 2 Humble、Nav2、Go2、MID-360、約1%の勾配。

## 結論と確認範囲

**案Aを比較試験用に採用する。Local CostmapとBehavior Serverをmap基準へ切り替え、3Dローカリゼーション・センサーTF・高さフィルタは初回試験では変更しない。** 本教材にあるのは起動例・補助ファイルであり、稼働中のGo2の設定やログではない。実機、ROS環境、rosbagはこの作業環境にない。高さ不一致の実測原因、実際のTF発行者、走行安定性、受け入れ条件の達成は未確認である。

参照先は教材のDockerfileとセットアップ章が指定する `rsasaki0109/lidar_localization_ros2`。今回の取得版は **21b3c9c62f590f9b04dd0a345ef779ab472f74eb**。実機の版と一致するか最初に確認する。Nav2は公開Humbleブランチの実装を参照した。インストール済みdebの版も試験記録へ残す。

## コードと設定で確認した事実

以下のパスと行は上記localizerコミットに対するもの。

| 箇所 | 確認内容 |
|---|---|
| `src/component_pose_publisher.cpp:62–114` | `publishPoseTransform`から`publishMapToOdomTransform`へ進み、姿勢の時刻でodom→baseをTF取得してmap→odomを配信。TF取得待ちは0.1 s。`publish_tf: false`なら配信しない |
| `include/lidar_localization/pose_publish_policy.hpp:174–210` | `T_map_odom = T_map_base × inverse(T_odom_base)`。通常は6DoFを合成する |
| 同ヘッダの`level`分岐 | 補正回転をYawのみにし、`t_map_odom = t_map_base − R_yaw × t_odom_base`と並進を再計算。現在のbase位置は保つが、姿勢のRoll/Pitch一致は保たない。Z並進も残る |
| `src/component_lifecycle.cpp:24–26,161–162` | `publish_tf=true`、`level_map_to_odom=false`、タイマー配信false、周期指定10 Hzが宣言デフォルト |
| `src/component_subscribers.cpp:126`、`src/component_pose_publisher.cpp` | タイマー有効時は設定周期でpublish。受理した照合による更新と、保持した補正の再配信は異なる。TFの配信Hzを自己位置補正の更新Hzと見なさない |
| `param/mid360_legged.yaml:106–107` | タイマー配信true、20 Hz。統合launchの既定はこのpresetではなく`nav2_ndt_urban.yaml`（周期指定30 Hz）。実効設定を取得して判断する |
| `launch/nav2_navigation.launch.py:66–75` | **入力YAMLを再生成しLocalをodom_frame_idへ上書きする**。Global・BTはglobal_frame_id、Behavior Serverはodom_frame_id。YAMLだけの案Aでは反映されない |
| 同launchの`publish_*_odom`引数・ノード | identity、twist、cmd_vel、localization由来のodom補助発行は既定false。実Go2のodom発行元はこのコードだけでは決定できない。複数を有効にしない |
| `param/nav2_humble_pointcloud.yaml:28–160` | DWB 5 Hz、Local mapではなくodom基準、6 m四方・rolling=true・更新2 Hz/公開1 Hz。Globalはmap基準・更新/公開0.5 Hz。robot_base_frameは両方base_link |
| 同preset | SimpleGoalCheckerはXY 0.25 m・Yaw 0.25 rad、stateful=true。DWBとCostmapのtransform_toleranceは1.5 s。大きい許容値で古いTFを隠さず、実際の遅延を測る |
| 同presetのLocal VoxelLayer | origin_z=0、z_resolution=0.1、z_voxels=16。Z格子範囲は概ね0～1.6 m。点群フィルタは−2～3 mでも、3 mまでボクセルがあるわけではない |
| Global ObstacleLayer | 点群高さ範囲−2～3 m。StaticLayerとInflationLayerも使用 |

教材内の`mid360_handheld.yaml`と同名launchは`use_odom=false`、`enable_map_odom_tf=false`、base=livox_frameの**手持ち構成**であり、Go2の実効設定として扱わない。統合launchの既定baseはbase_link。base_footprintへの変換やGo2ドライバの重力整合性は未確認。実機のURDF・odomノードを確認せずbase_footprintへ置き換えない。

## 高さずれと実処理への影響を分ける

Nav2のCostmap2DPublisherはOccupancyGridを各global_frameのorigin.z=0で公開する。map固定のRVizでは、odomの原点・軸がmapからずれればLocalの平面もずれる。回転がYawだけなら高低差はmap→odomのZ並進、Roll/Pitchもあれば位置によって高低差が変わる。点(x,y,0)のmapでの高さは`R31*x + R32*y + tz`。1度の傾きは100 m先で約1.75 mに相当する。

ただしZの見え方だけでは故障と断定できない。次を別々に測る。

- **表示**：両OccupancyGridのheader.frame_idとinfo.origin、同時刻のTF、RViz Fixed Frameを確認する。公開周期と更新周期は異なる。
- **障害物**：ObservationBufferは観測時刻のTFで点群をCostmap座標へ変換した後、そのZで高さを選別する。高さ閾値は床面からの相対高さではない。map基準へ変えると採用される点が変わり得る。
- **追従**：ロボット姿勢とGlobal PathをControllerのCostmap座標へ変換する。純Z並進だけならXY/Yawは変わらないが、Roll/Pitchを伴う3D変換と2D化は順序に依存し、経路や姿勢のXYに影響し得る。
- **ゴール**：SimpleGoalCheckerはXY距離とYaw差を判定し、Zを直接比較しない。TF不成立、XY投影の誤差、進捗判定、障害物、速度制限は別の未達原因。高さ表示とゴール未達の因果関係は未確認。

Rolling WindowはXYの窓を移動させる。床の高度に合わせてVoxelLayerのorigin_zを自動追従させる機能ではない。約1%でも100 mで1 m高度が変わるので、センサー原点と必要な障害物がZ格子内にあるかを走行全域で確認する。床誤検出は現在発生していないため、初回から床除去や高さ閾値を変更しない。

## Global Pathから局所目標を選ぶ仕組み

**Local Costmapが独自にゴールを作るわけではない。** ControllerがGlobal Pathの近傍部分を取り出し、局所的な追従・評価の対象にする。DWBでは「局所評価の目標」「最終ゴール」「選択した予測軌道の終端」を区別する。

| 対象 | DWB・Humbleでの意味 |
|---|---|
| 最終ゴール | FollowPathで受け取ったPath全体の最後の姿勢。Controller Serverが保存し、Local Costmapの座標へ変換してGoalCheckerへ渡す |
| 変換済みの近傍経路 | Global Pathから現在位置付近を切り出し、Local Costmapのglobal_frameへ変換した経路。`/transformed_global_plan`として可視化できる（公開設定による） |
| GoalDistの局所目標 | 近傍経路を格子解像度に合わせて補間し、Costmap内かつ未知セルでない点が続く区間の最後のセル。経路全体の最終ゴールとは限らない |
| `/local_plan`の終端 | 選んだ速度で短時間進むと予測される位置。局所目標と一致する必要はなく、最終到達判定にも使わない |

HumbleのDWBの処理は次の順序になる。

1. ロボット姿勢をGlobal Pathの座標系へ変換する。
2. Costmapの長い辺の半分を距離上限の基準とし、`prune_plan`・`prune_distance`で通過済みの経路を整理する。`shorten_transformed_plan`が有効なら、前方範囲は`forward_prune_distance`でも制限する。
3. 近傍の経路点をLocal Costmapの座標系へ変換し、Criticへ渡す。これは矩形の窓との厳密な交点を求める処理ではなく、経路点とロボットの距離による切り出しである。
4. `GoalDist`は経路上を順に見て、Costmap内の既知セルに入った後、範囲外または未知セルへ出る直前のセルを距離評価の起点にする。未知セルでないことは衝突しないことを意味せず、障害物評価は別のCriticが担当する。`PathDist`などは経路全体への追従も評価し、`GoalAlign`は前方評価点を考慮する。
5. 候補軌道を評価して速度を選び、次の周期に同じ処理を繰り返す。最終ゴールが窓の外でも、その方向へ続く近傍経路を追える。

例えば6 m四方のLocal Costmapなら距離上限の基準は3 m。前方切り詰めが有効で`forward_prune_distance=2 m`なら、近傍経路は原則2 mの距離条件で切られる。局所目標が必ず窓の端や最終ゴールになるわけではない。経路のサンプル間隔、未知領域、設定で選ばれる点は変わる。対応する点がなければDWBは経路変換・評価に失敗し、窓外に無理にゴールを作らない。

**最終到達判定は別処理である。** Controller ServerはPath全体の終端と現在姿勢を同じ座標系へ揃え、SimpleGoalCheckerでXY距離とYaw差を判定する。Z差そのものは判定しない。`stateful: true`では一度XY条件を満たした後、姿勢合わせの段階でXYの再確認を省く。局所目標や`/local_plan`の端へ到達しただけではNavigateToPoseの成功にはならない。

Local Costmapの`global_frame`をodomからmapへ変更すると、この変換先がmapになる。Global Pathがmap基準ならフレーム間の経路変換は不要になるが、ロボット姿勢・センサー点群には引き続きTFが必要である。純粋なZ並進だけならXY・Yawは変わらない。一方、Roll/Pitchを含む変換では3Dの変換と2Dへの投影の順序がXYへ影響し得るため、高さ表示だけで追従の正常性を判断しない。

RPPは先読み距離に基づく追従点、MPPIは予測区間の評価を使うなど、局所の目標の扱いはControllerごとに異なる。上記のGoalDistのセル選択をNav2全体の共通仕様と見なさない。

参考：[DWBの経路変換](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)、[GoalDistの局所目標選択](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/goal_dist.cpp)、[Controller Serverの最終ゴール判定](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp)。

## 案の比較

| 案 | 効果 | リスク・採否 |
|---|---|---|
| A Localをmapへ | Global/Localの表示平面と基準軸が一致。6DoF TFを維持 | 補正ジャンプ時に格子内の過去障害物が残る、クリア不整合、TF依存増加、高さ判定変更。**比較試験として採用、実機採用は未確定** |
| B level_map_to_odom=true | 補正のRoll/Pitchを除きYawを維持。現在base位置に合うよう並進を再計算 | Zずれは解消しない。重力整合したodomが前提。LiDAR登録姿勢とTFの姿勢は相違し得る。現状はfalseを保ち、独立試験 |
| C Nav2専用2D座標系 | 3D状態と2D制御の責務を分離できる | Path・姿勢・センサー点群の投影整合、TF所有者、親子関係の設計が必要。Aで問題が実証された場合だけ検討。新しいBroadcasterは実装しない |

## 最小修正と適用

成果物`nav2_local_costmap_frame.patch`は上記コミットの統合launchに適用する通常のunified diff。`local_costmap_frame`引数を追加し、空なら従来どおりodom_frame_idを使う。map指定時はLocalと、それを利用するBehavior Serverのglobal_frameを揃える。Global、BT、センサーTF、localizerの6DoF、odom_frame_idは変更しない。YAMLの高さ設定も保持する。

```bash
# 実機用パッケージのソースで実行。PATCHはこの教材内のpatchへの絶対パス。
git rev-parse HEAD
git status --short
git apply --check "$PATCH"
git apply "$PATCH"
# 既存ワークスペースの手順で再ビルドしてinstall/setup.bashをsourceする。
# 通常の起動引数にこれを追加する（カスタムmap名ならその名前）：
# local_costmap_frame:=map
# 対照試験・切り戻し：local_costmap_frame:=odom
```

`odom_frame_id:=map`で代用しない。localizerのTF契約まで変わってしまう。既存のYAML全体を小さな断片で置換しない。patchが適用できない版では強制適用せず、同じ書換処理の有無を確認する。上流リポジトリ自体にはPushしていない。

ROSなしの補助検証：`python3 check_nav2_local_costmap_frame.py /path/to/patched/package`（PyYAMLが必要）。デフォルト・map・カスタムフレーム、Behaviorとの整合、Rolling Window、高さ設定保持を実際の書換関数で確認する。patchの適用チェックも合格。ROSビルド・launch起動・走行試験は未実施。

## 実機とrosbagでの検証手順

1. **現状を保存**：localizer、controller_server、両Costmap、behavior_serverの`ros2 param dump`、起動引数、URDF、git SHA、deb版、map YAMLを保存する。`ros2 node list`で名前を解決し、`level_map_to_odom`、`publish_tf`、タイマー設定、実効フレームを取得する。
2. **TF所有者**：以下を停止中・走行中・勾配上で記録する。`ros2 topic info /tf --verbose`は候補publisherを示すが、個々のTF辺の所有者の証明にはならない。起動設定、ノード実装、必要ならpublisherを一つずつ停止して、map→odom・odom→base・base→lidar各辺を一系統に特定する。

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_monitor map base_link
ros2 topic info /tf --verbose
ros2 topic echo /odom --once
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once
ros2 param get /local_costmap/local_costmap global_frame
ros2 param get /global_costmap/global_costmap global_frame
```

3. **対照と案A**：まず従来設定、その後`local_costmap_frame:=map`で同じ経路・障害物・速度を試す。実効LocalとBehaviorのglobal_frameがmapか確認。最初は駆動を切り、平坦路、低速平坦走行、1%上り/下り、停止・再開、旋回、ゴール付近の順に進める。再ローカライズは駆動を切って実施する。
4. **収録**：実トピック名に合わせ、`/tf /tf_static /odom /livox/lidar /livox/imu /map /plan /local_plan /transformed_global_plan /local_costmap/costmap /global_costmap/costmap /cmd_vel /rosout`、実localizer姿勢、各段の速度指令、FollowPath/NavigateToPoseのstatusを記録する。サービス/actionの結果はクライアント側でも保存する。存在しないトピックやDWBの非公開デバッグ出力は先に確認する。
5. **リプレイ**：use_sim_time=true、`ros2 bag play BAG --clock`。記録済みTFでNav2だけ比較する試験と、センサー・odomからlocalizerも再実行する試験を分ける。後者ではbag内のmap→odomと稼働localizerを同時配信しない。/tfには複数辺が混在するので、必要な辺だけを残す事前処理を行う。駆動先へcmd_velを送らない。bagは閉ループ走行の成功を保証しない。
6. **案Bは別実験**：odomのZ/Roll/Pitchと重力整合性を確認した後だけ、停止状態でlocalizer YAMLのlevel_map_to_odomを変更して再起動する。Aの試験結果と混ぜない。

## 変更前後の比較記録と受け入れ判定

| 項目 | 従来の構成からの予測 | 案Aでの予測 | 実測状態 |
|---|---|---|---|
| Costmap平面 | map/odomの変換で高低差・傾斜が見える | 同じmap平面 | 両方未測定 |
| 3D TF | 6DoF補正 | 同じ補正式、Z保持 | 未測定 |
| 局所障害物 | odom基準で保持 | map補正時に過去の点とずれる可能性 | 未測定 |
| Zフィルタ・ボクセル | odomの高さで評価 | mapの高さで評価 | 未測定 |
| 追従・ゴール | TF変換後のXY/Yawを使用 | map上のXY/Yawを使用 | 未測定 |

試験前に停止距離・許容遅延・横偏差・残像消失時間の数値基準を実機担当と定める。各ケースで最大TF遅延・欠落、ローカリゼーション補正前後のXY/Z/RPY差、障害物の検出率/消失時間、Costmap更新間隔、Controller失敗回数、最終XY/Yaw誤差とaction結果を比較する。

受け入れは、平坦路と1%勾配での走行成功、3D地形高度の保持、障害物検出・回避の非劣化、更新安定、TF重複なし、ゴール判定成功を実測してからとする。localizerのmap→odomは本来補正で不連続になり得るため、「不連続なし」は根拠なく保証せず、意図しないジャンプがないことと補正時の安全な停止/復帰を判定する。

AでZ格子外への欠落があれば、走行域の最低/最高高度と必要な障害物高さからorigin_z・z_resolution・z_voxels・高さフィルタを再設計する。HumbleのVoxelGridは最大16層なので、広い高度範囲には分解能とのトレードオフがある。記録された欠落なしに範囲を広げない。補正後の残像はClearによる一時改善だけで合格とせず、走行中の再現性を確認する。これらで実用性が得られない場合だけ案Cへ進む。

## 参照元

- [localizer 調査コミット](https://github.com/rsasaki0109/lidar_localization_ros2/tree/21b3c9c62f590f9b04dd0a345ef779ab472f74eb)
- [Nav2 Humble DWB](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)
- [GoalDistCritic](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/goal_dist.cpp)
- [Controller Server](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp)
- [SimpleGoalChecker](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/plugins/simple_goal_checker.cpp)
- [ObservationBuffer](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/observation_buffer.cpp)
