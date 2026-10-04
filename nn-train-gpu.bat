@echo off
rem Train the neural-net driver in the GPU simulator: thousands of simplified matches at once
rem on your NVIDIA card. Ctrl+C saves and stops; nn-train-gpu.bat --resume continues.
rem Afterwards fine-tune it in the real game: nn-train.bat --resume --level 3
rem Options: tools\nn\train_gpu.py --help (e.g. --envs 8192 for more matches at once).
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
.venv\Scripts\python tools\nn\train_gpu.py %*
pause
