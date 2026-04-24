@echo off
echo.
cd /d "%~dp0"

python -c "import flask" 2>nul
if errorlevel 1 (
    echo Installing Flask...
    python -m pip install -r requirements.txt
)

set PORT=25565
if "%ROBOT_API_TOKEN%"=="" set ROBOT_API_TOKEN=robot123
rem Cloudflare tunnel starts automatically if cloudflared is installed.
rem To use a manually-started tunnel instead: set CLOUDFLARE_URL=https://your-tunnel.trycloudflare.com
python app.py
pause
