@echo off
cd /d "%~dp0"
docker compose -f compose.dev.yaml ps
pause
