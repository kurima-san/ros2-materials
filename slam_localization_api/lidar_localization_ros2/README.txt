Livox MID-360 + GLIM + lidar_localization_ros2 Complete Guide v3

Open:
  index.html

Major chapters:
  01 Architecture
  02 Livox setup
  03 GLIM setup / mapping
  04 lidar_localization_ros2 build/setup
  05 2D map generation, gray-map diagnosis, floor/slope-aware conversion
  06 Launch / RViz
  07 Realtime localization
  08 rosbag localization
  09 Recovery
  10 Diagnostics
  11 Nav2 navigation, slopes/stairs/multi-floor
  12 Automatic relocalization
  13 References

Included helper:
  tools/generate_floor_aware_occupancy_map.py

Included fixes:
  fix/CMakeLists.txt
  fix/ndt_omp_target_link_fix.patch
  fix/nav2_navigation_mid360_imu.patch
  fix/nav2_robot_interface_checklist.txt

MID-360 mapping pre-filter added:
  tools/mid360_mapping_filter.py
  tools/run_mid360_mapping_filter_example.sh

Purpose:
- software minimum/maximum range filter
- sensor carrier exclusion box
- companion-person exclusion box
- publish /livox/lidar_mapping for GLIM

Important:
- MID360_config.json does not expose HAP's blind_spot_set parameter.
- lidar_localization_ros2 scan_min_range is for localization, not GLIM mapping.
