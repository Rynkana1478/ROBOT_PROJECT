@echo off
echo ================================================
echo   4WD Robot Server - Port 25565
echo   Public IP: Check at ifconfig.me
echo   Dashboard: http://localhost:25565
echo   Token: robot123
echo ================================================
echo.

cd /d "%~dp0"
"path" -c "import flask" 2>nul
if errorlevel 1 (
    echo Installing Flask...
    "path" -m pip install flask
)

set PORT=25565
set ROBOT_API_TOKEN=robot123
"path" app.py
pause
