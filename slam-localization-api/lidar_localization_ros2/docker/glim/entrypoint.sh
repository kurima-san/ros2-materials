#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
if [[ ! -d /data/config/glim && -w /data/config ]]; then
  mkdir -p /data/config/glim
fi
if [[ ! -f /data/config/glim/config_ros.json && -w /data/config/glim ]]; then
  cp -a /opt/ros/humble/share/glim/config/. /data/config/glim/
fi
exec "$@"
