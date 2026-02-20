@echo off
echo ========================================
echo   RC Boat Control System
echo ========================================
echo.

:: Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    pause
    exit /b 1
)

:: Check if Node.js is installed
node --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Node.js is not installed or not in PATH
    pause
    exit /b 1
)

echo Starting Backend Server...
start "RC Boat Backend" cmd /k "cd /d %~dp0backend && python main.py"

:: Wait for backend to start
timeout /t 3 /nobreak >nul

echo Starting Frontend Server...
start "RC Boat Frontend" cmd /k "cd /d %~dp0frontend && npm run dev"

:: Wait for frontend to start
timeout /t 5 /nobreak >nul

echo.
echo ========================================
echo   Application Started!
echo   Open http://localhost:3000 in browser
echo ========================================
echo.

:: Open browser
start http://localhost:3000

echo Press any key to close this window...
pause >nul

