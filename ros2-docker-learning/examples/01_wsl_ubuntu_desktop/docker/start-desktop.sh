#!/usr/bin/env bash
set -e
export DISPLAY=:1
mkdir -p "$HOME/.vnc"
printf '%s\n' "${VNC_PASSWORD:-ros2}" | vncpasswd -f > "$HOME/.vnc/passwd"
chmod 600 "$HOME/.vnc/passwd"
tigervncserver -kill :1 >/dev/null 2>&1 || true
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
tigervncserver :1 -localhost yes -geometry "${VNC_GEOMETRY:-1600x900}" -depth 24 -SecurityTypes VncAuth -xstartup /usr/local/bin/vnc-xstartup
websockify --web=/usr/share/novnc/ 6080 localhost:5901 > /tmp/websockify.log 2>&1 &
if [ "${AUTO_START_RVIZ:-1}" = "1" ]; then
  (sleep 5; source "/opt/ros/${ROS_DISTRO}/setup.bash"; DISPLAY=:1 rviz2 -d /workspace/config/fake_lidar.rviz > /tmp/rviz.log 2>&1) &
fi
cleanup(){ tigervncserver -kill :1 >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM
wait
