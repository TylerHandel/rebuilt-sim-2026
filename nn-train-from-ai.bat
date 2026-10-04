@echo off
rem Train a new neural network that first copies the pre-programmed AIs (recorded with
rem nn-record-ai.bat), then keeps improving on the GPU, imitating them a little less as it goes.
rem It's a separate run (runs\from-ai), so your current network in runs\driver stays as it is;
rem pick the run on the dashboard to compare. Running it again continues. Ctrl+C saves and stops.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
if not exist recordings-ai (echo No recorded AI matches yet: run nn-record-ai.bat first. & pause & exit /b 1)
set RESUME=
if exist runs\from-ai\ckpt.pt set RESUME=--resume
.venv\Scripts\python tools\nn\train_gpu.py --run from-ai %RESUME% --bc recordings-ai --compile %*
pause
