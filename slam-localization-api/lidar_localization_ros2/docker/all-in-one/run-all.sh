#!/usr/bin/env bash
set -euo pipefail

pids=()
stop_all() {
  trap - INT TERM EXIT
  ((${#pids[@]})) && kill "${pids[@]}" 2>/dev/null || true
  wait || true
}
trap stop_all INT TERM EXIT

ros2 launch livox_ros_driver2 rviz_MID360_launch.py \
  user_config_path:=/data/config/MID360_config.json &
pids+=("$!")

if [[ "${ENABLE_GLIM:-true}" == "true" ]]; then
  ros2 run glim_ros glim_rosnode \
    --ros-args -p config_path:=/data/config/glim &
  pids+=("$!")
fi

if [[ "${ENABLE_LOCALIZATION:-true}" == "true" ]]; then
  ros2 launch /opt/guide/mid360_handheld_localization.launch.py \
    localization_param_dir:=/data/config/localization.yaml \
    start_rviz:="${LOCALIZATION_RVIZ:-false}" &
  pids+=("$!")
fi

# いずれかが異常終了したら残りも止め、コンテナを失敗終了させる。
set +e
wait -n "${pids[@]}"
status=$?
set -e
stop_all
exit "$status"
