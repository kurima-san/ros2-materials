#!/usr/bin/env bash
set -euo pipefail

pipeline=${PIPELINE:-}
input_mode=${INPUT_MODE:-realtime}
sensor_model=${SENSOR_MODEL:-mid360s}
pids=()

csv_to_array() {
  local value=$1
  local -n output=$2
  local item
  IFS=',' read -r -a output <<< "$value"
  for item in "${output[@]}"; do
    [[ -n "$item" && "$item" != *[[:space:]]* ]] || {
      echo >&2 "Topic lists must be comma-separated, non-empty names without spaces"
      exit 2
    }
  done
}

case "$pipeline" in
  mapping|localization) ;;
  *) echo >&2 "PIPELINE must be 'mapping' or 'localization'"; exit 2 ;;
esac
case "$input_mode" in
  realtime|bag) ;;
  *) echo >&2 "INPUT_MODE must be 'realtime' or 'bag'"; exit 2 ;;
esac
case "$sensor_model" in
  mid360) sensor_config=/data/config/MID360_config.json ;;
  mid360s) sensor_config=/data/config/MID360s_config.json ;;
  *) echo >&2 "SENSOR_MODEL must be 'mid360' or 'mid360s'"; exit 2 ;;
esac

stop_all() {
  trap - INT TERM EXIT
  ((${#pids[@]})) && kill "${pids[@]}" 2>/dev/null || true
  wait || true
}
trap stop_all INT TERM EXIT

if [[ "$input_mode" == realtime ]]; then
  ros2 launch livox_ros_driver2 rviz_MID360_launch.py \
    user_config_path:="$sensor_config" &
  pids+=("$!")
else
  if [[ -z "${BAG_PATH:-}" || ! -e "${BAG_PATH}" ]]; then
    echo >&2 "For INPUT_MODE=bag, set BAG_PATH to a bag under /data/bags"
    exit 2
  fi
  play_args=("${BAG_PATH}" --clock)
  if [[ -n "${PLAY_TOPICS:-}" ]]; then
    csv_to_array "$PLAY_TOPICS" play_topics
    play_args+=(--topics "${play_topics[@]}")
  fi
  ros2 bag play "${play_args[@]}" &
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
  csv_to_array "${RECORD_TOPICS:-/livox/lidar,/livox/imu,/tf,/tf_static}" record_topics
  output="/data/bags/${pipeline}_$(date -u +%Y%m%dT%H%M%SZ)"
  ros2 bag record -o "$output" "${record_topics[@]}" &
  pids+=("$!")
  echo "Recording topics to $output: ${record_topics[*]}"
fi

# Any component ending ends the complete sensor/playback + processing pipeline.
set +e
wait -n "${pids[@]}"
status=$?
set -e
stop_all
exit "$status"
