MOLA + Livox Mid-360 complete bundle
====================================

このbundleは、MOLA本体のYAML/launch/sourceを手で部分編集せずに使うための追加ファイル一式です。

1) 既にMOLA source buildが完了している場合:

   cd /path/to/mola_mid360_complete_bundle
   ./setup/install_mola_shell.sh
   source ~/.bashrc
   ./install_tools.sh
   mola_shell

2) Realtime localization:

   ros2 run mola_mid360_tools run_realtime_localization ~/mola_maps/atr_out_lc_clean.mm
   # RViz 2D Pose Estimate
   ros2 run mola_mid360_tools resume_localization

3) rosbag localization:

   Terminal 1:
   ros2 run mola_mid360_tools run_rosbag_localization ~/mola_maps/atr_out_lc_clean.mm

   Terminal 2:
   ros2 run mola_mid360_tools play_rosbag_paused ~/workspace/ros_bag/atr_localization_test/

4) 位置ずれ復旧:

   ros2 run mola_mid360_tools pause_localization
   # RViz 2D Pose Estimate
   ros2 run mola_mid360_tools resume_localization

詳細は mola_mid360_setup_localization_guide.html を開いてください。
