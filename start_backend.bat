@echo off
REM ============================================
REM Backend Server Only - Development Mode
REM ============================================

echo.
echo ================================================
echo   Starting Backend API Server
echo ================================================
echo.

REM Check if .env exists
if not exist .env (
    echo [ERROR] .env file not found!
    echo Please copy .env.example to .env and configure your API keys.
    echo.
    pause
    exit /b 1
)

echo Installing dependencies...
pip install -r backend/requirements.txt

echo.
echo Starting backend server on http://localhost:8000
echo API Documentation: http://localhost:8000/docs
echo.

REM Run backend server
python backend/src/api/main.py

pause
