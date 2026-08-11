cmake_minimum_required(VERSION 3.8)
project(mola_mid360_tools)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_realtime_localization
  scripts/run_rosbag_localization
  scripts/play_rosbag_paused
  scripts/pause_localization
  scripts/resume_localization
  scripts/map_rosbag_to_simplemap
  scripts/run_loop_closure
  scripts/build_metric_map
  scripts/view_metric_map
  scripts/check_mid360_topics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
