#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
URL="http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale"
if grep -qi microsoft /proc/version 2>/dev/null; then
  powershell.exe -NoProfile -Command "Start-Process '$URL'" >/dev/null 2>&1 &
else
  xdg-open "$URL" >/dev/null 2>&1 &
fi
docker compose up --build
