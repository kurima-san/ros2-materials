@echo off
cd /d "%~dp0"
echo [1/2] 開発用Containerを作成・起動します。
docker compose -f compose.dev.yaml up -d --build
if errorlevel 1 goto error
echo [2/2] ブラウザDesktopを開きます。
timeout /t 2 /nobreak >nul
start "" "http://127.0.0.1:6080/vnc.html?autoconnect=1^&resize=scale"
echo.
echo VNC password: ros2
echo Container Terminal: open-terminal.bat
echo 状態確認: status-dev.bat
echo 終了: stop-dev.bat
pause
exit /b 0
:error
echo Dockerの起動に失敗しました。
pause
exit /b 1
