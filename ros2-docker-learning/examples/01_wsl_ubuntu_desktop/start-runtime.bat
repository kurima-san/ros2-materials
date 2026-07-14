@echo off
cd /d "%~dp0"
docker compose -f compose.runtime.yaml up --build
pause
