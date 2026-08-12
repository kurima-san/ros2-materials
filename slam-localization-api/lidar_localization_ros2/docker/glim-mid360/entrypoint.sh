#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/livox_ws/install/setup.bash

# 初回起動時だけ編集用の設定をvolumeへ用意する。既存設定は上書きしない。
if [[ ! -f /data/config/MID360_config.json && -w /data/config ]]; then
  cp /opt/mid360_defaults/MID360_config.json /data/config/MID360_config.json
fi
if [[ ! -d /data/config/glim && -w /data/config ]]; then
  mkdir -p /data/config/glim
  cp -a /opt/ros/humble/share/glim/config/. /data/config/glim/
fi

exec "$@"
