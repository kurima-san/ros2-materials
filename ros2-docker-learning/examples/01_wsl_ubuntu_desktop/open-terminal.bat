@echo off
cd /d "%~dp0"
echo desktop Container内のbashを開きます。終了は exit です。
docker compose -f compose.dev.yaml exec desktop bash
pause
