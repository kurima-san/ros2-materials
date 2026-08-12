#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/livox_ws/install/setup.bash
if [[ ! -f /data/config/MID360_config.json && -w /data/config ]]; then
  cp /opt/mid360_defaults/MID360_config.json /data/config/MID360_config.json
fi
exec "$@"
