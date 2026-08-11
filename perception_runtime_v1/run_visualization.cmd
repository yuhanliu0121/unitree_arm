@echo off
setlocal
cd /d "%~dp0"
set "PYTHON_EXE=C:\Users\breez\Documents\New project\.venv-realsense\Scripts\python.exe"
set "YOLO_CONFIG_DIR=%~dp0.runtime_cache\ultralytics"

if not exist "%PYTHON_EXE%" (
  echo [ERROR] Python not found: %PYTHON_EXE%
  pause
  exit /b 1
)
if not exist "%~dp0weights\best.pt" (
  echo [ERROR] Frozen weight not found: %~dp0weights\best.pt
  pause
  exit /b 1
)

echo ============================================================
echo  Go2 RGB-D Perception Runtime V1
echo  1 = DOG navigation profile
echo  2 = ARM grasping profile
echo ============================================================
choice /C 12 /N /M "Choose profile [1/2]: "
if errorlevel 2 (
  set "PROFILE=arm"
) else (
  set "PROFILE=dog"
)

echo.
echo Starting %PROFILE% visualization...
echo Q/Esc quit  S save  D switch view  R rejected  Space pause
"%PYTHON_EXE%" -m perception_runtime.app ^
  --config "%~dp0configs\runtime.yaml" ^
  --profile "%PROFILE%" ^
  --view split ^
  --output "%~dp0captures"
echo.
echo Exited with code %errorlevel%
pause
endlocal
