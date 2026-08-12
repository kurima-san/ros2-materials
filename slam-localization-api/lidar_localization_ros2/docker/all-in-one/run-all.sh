#!/usr/bin/env bash
set -euo pipefail

pipeline=${PIPELINE:-}
input_mode=${INPUT_MODE:-realtime}
pids=()

case "$pipeline" in
  mapping|localization) ;;
  *) echo >&2 "PIPELINE must be 'mapping' or 'localization'"; exit 2 ;;
esac
case "$input_mode" in
  realtime|bag) ;;
  *) echo >&2 "INPUT_MODE must be 'realtime' or 'bag'"; exit 2 ;;
esac

stop_all() {
  trap - INT TERM EXIT
  ((${#pids[@]})) && kill "${pids[@]}" 2>/dev/null || true
  wait || true
}
trap stop_all INT TERM EXIT

if [[ "$input_mode" == realtime ]]; then
  ros2 launch livox_ros_driver2 rviz_MID360_launch.py \
    user_config_path:=/data/config/MID360_config.json &
  pids+=("$!")
else
  if [[ -z "${BAG_PATH:-}" || ! -e "${BAG_PATH}" ]]; then
    echo >&2 "For INPUT_MODE=bag, set BAG_PATH to a bag under /data/bags"
    exit 2
  fi
  ros2 bag play "${BAG_PATH}" --clock &
  pids+=("$!")
fi

if [[ "$pipeline" == mapping ]]; then
  ros2 run glim_ros glim_rosnode --ros-args \
    -p config_path:=/data/config/glim \
    -p use_sim_time:="$([[ "$input_mode" == bag ]] && echo true || echo false)" &
else
  ros2 launch /opt/guide/mid360_handheld_localization.launch.py \
    localization_param_dir:=/data/config/localization.yaml \
    use_sim_time:="$([[ "$input_mode" == bag ]] && echo true || echo false)" \
    start_rviz:="${LOCALIZATION_RVIZ:-true}" &
fi
pids+=("$!")

if [[ "${RECORD_BAG:-false}" == true ]]; then
  output="/data/bags/${pipeline}_$(date -u +%Y%m%dT%H%M%SZ)"
  ros2 bag record -o "$output" /livox/lidar /livox/imu /tf /tf_static &
  pids+=("$!")
  echo "Recording input and TF topics to $output"
fi

# Any component ending ends the complete sensor/playback + processing pipeline.
set +e
wait -n "${pids[@]}"
status=$?
set -e
stop_all
exit "$status"
