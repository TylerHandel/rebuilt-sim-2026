@echo off
rem Continue training the neural-net driver on the GPU with the settings suited to an already
rem trained network: resumes runs\driver, fused GPU kernels (--compile), and a mix of mostly
rem older versions of itself (it plays the ones it loses to more often), itself, and the
rem scripted bots (some of which defend). Ctrl+C saves and stops. Double-click to run.
rem Extra options can be added after the name, e.g. nn-continue-gpu.bat --envs 8192
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (echo Run nn-setup.bat first. & pause & exit /b 1)
if not exist runs\driver\ckpt.pt (echo No training to continue in runs\driver. Start one with nn-train-gpu.bat, or run import-old.bat to bring it over. & pause & exit /b 1)
.venv\Scripts\python tools\nn\train_gpu.py --resume --selfplay --compile --mix 0 0.25 0.45 0.3 %*
pause
