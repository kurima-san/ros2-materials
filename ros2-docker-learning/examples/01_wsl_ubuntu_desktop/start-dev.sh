#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
URL="http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale"

echo "[1/2] 開発用Containerを作成・起動します。"
docker compose -f compose.dev.yaml up -d --build

echo "[2/2] ブラウザDesktopを開きます。"
sleep 2
if grep -qi microsoft /proc/version 2>/dev/null; then
  powershell.exe -NoProfile -Command "Start-Process '$URL'" >/dev/null 2>&1 &
elif command -v xdg-open >/dev/null 2>&1; then
  xdg-open "$URL" >/dev/null 2>&1 &
fi

echo
echo "URL: $URL"
echo "VNC password: ros2"
echo "Container Terminal: ./open-terminal.sh"
echo "状態確認: ./status-dev.sh"
echo "終了: ./stop-dev.sh"
