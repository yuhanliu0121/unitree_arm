@echo off
setlocal
cd /d "%~dp0"
set "PYTHON_EXE=C:\Users\breez\Documents\New project\.venv-realsense\Scripts\python.exe"
set "DATASET=G:\WorkSpace\机械狗感知\real_test_camera_v1"
set "STEM=cap_20260801_082106_0001"
set "YOLO_CONFIG_DIR=%~dp0.runtime_cache\ultralytics"

"%PYTHON_EXE%" -m perception_runtime.offline_demo ^
  --dataset "%DATASET%" ^
  --stem "%STEM%" ^
  --config "%~dp0configs\runtime.yaml" ^
  --profile dog ^
  --output "%~dp0offline_example"

echo.
echo Result: %~dp0offline_example
pause
endlocal
