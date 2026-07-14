@echo off
cd /d "%~dp0"
start "" "http://127.0.0.1:6080/vnc.html?autoconnect=1^&resize=scale"
docker compose up --build
pause
