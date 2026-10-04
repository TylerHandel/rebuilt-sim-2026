@echo off
rem Train the neural-net driver (GPU + all CPU threads). Ctrl+C saves and stops.
rem   nn-train.bat                      start a new run (or continue one: nn-train.bat --resume)
rem   nn-train.bat --bc recordings      learn from your recorded driving first
rem Any tools\nn\train.py option works here; see tools\nn\README.md.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
.venv\Scripts\python tools\nn\train.py %*
pause
