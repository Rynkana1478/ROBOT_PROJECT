@echo off
echo Killing old server...
taskkill /F /IM python.exe 2>nul
timeout /t 2 /noq >nul
echo Starting server on port 25565...
cd /d "%~dp0"
set PORT=25565
set DDNS_HOST=blackwise.thddns.net
set DDNS_PORT=5570
start "RobotServer" python app.py
echo Server restarted.
