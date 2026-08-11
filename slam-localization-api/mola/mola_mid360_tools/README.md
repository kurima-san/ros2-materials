# mola_mid360_tools

Livox Mid-360 + MOLA用の追加パッケージです。MOLA本体のファイルを編集せず、`~/mola_ws/src/` にこのフォルダを追加してbuildします。

## Install

```bash
cp -a mola_mid360_tools ~/mola_ws/src/
cd ~/mola_ws
source /opt/ros/humble/setup.bash
source ~/mola_ws/install/setup.bash 2>/dev/null || true
colcon build --symlink-install --packages-select mola_mid360_tools
```

新しいMOLA shellへ入り直した後に使用します。

## Realtime localization

```bash
ros2 run mola_mid360_tools run_realtime_localization ~/mola_maps/atr_out_lc.mm
```

RVizで2D Pose Estimateを指定したあと:

```bash
ros2 run mola_mid360_tools resume_localization
```

## rosbag localization

Terminal 1:

```bash
ros2 run mola_mid360_tools run_rosbag_localization ~/mola_maps/atr_out_lc.mm
```

Terminal 2:

```bash
ros2 run mola_mid360_tools play_rosbag_paused ~/workspace/ros_bag/atr_localization_test/
```

RVizで初期位置を指定し、`resume_localization`後にbag側でSpaceを押します。

## Recovery

```bash
ros2 run mola_mid360_tools pause_localization
# RViz: 2D Pose Estimate
ros2 run mola_mid360_tools resume_localization
```
