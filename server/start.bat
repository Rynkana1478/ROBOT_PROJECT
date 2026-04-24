@echo off
echo.
cd /d "%~dp0"

python -c "import flask" 2>nul
if errorlevel 1 (
    echo Installing Flask...
    python -m pip install -r requirements.txt
)

set PORT=25565
set DDNS_HOST=blackwise.thddns.net
set DDNS_PORT=5570
python app.py
pause
