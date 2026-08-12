#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/livox_ws/install/setup.bash
source /opt/localization_ws/install/setup.bash
if [[ ! -f /data/config/MID360_config.json && -w /data/config ]]; then
  cp /opt/stack_defaults/MID360_config.json /data/config/MID360_config.json
fi
if [[ ! -d /data/config/glim && -w /data/config ]]; then
  mkdir -p /data/config/glim
fi
if [[ ! -f /data/config/glim/config_ros.json && -w /data/config/glim ]]; then
  cp -a /opt/ros/humble/share/glim/config/. /data/config/glim/
fi
if [[ ! -f /data/config/localization.yaml && -w /data/config ]]; then
  cp /opt/guide/mid360_handheld.yaml /data/config/localization.yaml
fi
exec "$@"
