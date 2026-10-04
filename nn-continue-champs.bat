@echo off
rem Continue training the neural-net driver in the full game against the Champs-level AIs
rem (Scorer, Defense, Hybrid), 1v1 and 3v3, with its teammates driven by the network too.
rem Slower than the GPU trainer, but it learns the real physics. Ctrl+C saves and stops.
rem It first backs up your GPU training to runs\driver-gpu (the full-game trainer doesn't keep
rem the GPU trainer's pool of older versions). Double-click to run.
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
if not exist runs\driver\ckpt.pt (echo No training to continue in runs\driver. & pause & exit /b 1)
echo Backing up runs\driver to runs\driver-gpu ...
robocopy runs\driver runs\driver-gpu /E /NFL /NDL /NJH /NJS /NP >nul
.venv\Scripts\python tools\nn\train.py --resume --level 4 --teams 1 3 %*
pause
