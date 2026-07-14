@echo off
cd /d "%~dp0"
docker compose -f compose.jetson-sim.yaml up --build
pause
