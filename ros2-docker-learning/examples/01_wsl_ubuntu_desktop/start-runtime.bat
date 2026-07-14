@echo off
cd /d "%~dp0"
docker compose -f compose.runtime.yaml up -d --build
docker compose -f compose.runtime.yaml ps
pause
