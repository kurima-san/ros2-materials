#!/usr/bin/env bash
cd "$(dirname "$0")"
URL="http://127.0.0.1:8000"
if grep -qi microsoft /proc/version 2>/dev/null; then
  powershell.exe -NoProfile -Command "Start-Process '$URL'" >/dev/null 2>&1 &
else
  xdg-open "$URL" >/dev/null 2>&1 &
fi
python3 -m http.server 8000
