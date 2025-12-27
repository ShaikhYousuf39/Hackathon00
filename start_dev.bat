@echo off
REM ============================================
REM Development Startup Script
REM Starts both frontend and backend servers
REM ============================================

echo.
echo ================================================
echo   Physical AI RAG Chatbot - Development Mode
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

echo [1/4] Checking dependencies...
echo.

REM Check if Python is installed
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python is not installed or not in PATH
    pause
    exit /b 1
)

REM Check if Node.js is installed
node --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Node.js is not installed or not in PATH
    pause
    exit /b 1
)

echo [2/4] Installing backend dependencies...
echo.
pip install -r backend/requirements.txt
if errorlevel 1 (
    echo [ERROR] Failed to install backend dependencies
    pause
    exit /b 1
)

echo.
echo [3/4] Installing frontend dependencies...
echo.
cd frontend
call npm install
if errorlevel 1 (
    echo [ERROR] Failed to install frontend dependencies
    cd ..
    pause
    exit /b 1
)
cd ..

echo.
echo [4/4] Starting servers...
echo.
echo Backend will start on: http://localhost:8000
echo Frontend will start on: http://localhost:3000
echo.
echo Press Ctrl+C to stop all servers
echo.

REM Start backend server in new window
start "Backend Server" cmd /k "python backend/src/api/main.py"

REM Wait 3 seconds for backend to start
timeout /t 3 /nobreak >nul

REM Start frontend server
cd frontend
start "Frontend Server" cmd /k "npm start"
cd ..

echo.
echo ================================================
echo   Servers started successfully!
echo ================================================
echo.
echo Backend API: http://localhost:8000
echo Frontend App: http://localhost:3000
echo API Docs: http://localhost:8000/docs
echo.
pause
