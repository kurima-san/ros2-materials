#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
# source ~/livox_ws/install/setup.bash

# IMPORTANT:
# The box coordinates below are EXAMPLES only.
# Display /livox/lidar in RViz using livox_frame and measure the actual location
# of the sensor carrier / companion before setting these values.

python3 "$(dirname "$0")/mid360_mapping_filter.py" --ros-args \
  -p input_topic:=/livox/lidar \
  -p output_topic:=/livox/lidar_mapping \
  -p min_range_m:=0.50 \
  -p max_range_m:=0.0 \
  -p self_box_enabled:=false \
  -p companion_box_enabled:=true \
  -p companion_min_x:=-1.5 \
  -p companion_max_x:=0.5 \
  -p companion_min_y:=-1.5 \
  -p companion_max_y:=-0.3 \
  -p companion_min_z:=-1.5 \
  -p companion_max_z:=1.0
